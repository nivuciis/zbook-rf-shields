/*
 * Gravador do CH592: fala com o bootloader (ROM) do chip pela uart1, no
 * protocolo ISP serial da WCH. Pedido e resposta, um comando por vez, tudo
 * por polling (uart_poll_out / uart_poll_in).
 *
 * Sequencia (ch592_isp_flash):
 *   IDENTIFY -> READ_CONFIG (UID) -> ISP_KEY (chave XOR) -> ERASE
 *   -> PROGRAM (blocos de 56 B) -> VERIFY -> ISP_END (roda o firmware novo)
 *
 * Mapa do arquivo:
 *   isp_xfer()          monta e envia um frame, le e confere a resposta
 *   ch592_isp_connect() IDENTIFY ate o bootloader responder
 *   isp_session()       READ_CONFIG + ISP_KEY (calcula a chave)
 *   isp_resync()        refaz a sessao depois de um bloco perdido
 *   write_chunks()      PROGRAM ou VERIFY, bloco a bloco
 *   ch592_isp_flash()   a sequencia completa
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "ch592_isp.h"

/* Comandos do bootloader */
#define CMD_IDENTIFY    0xA1
#define CMD_ISP_END     0xA2
#define CMD_ISP_KEY     0xA3
#define CMD_ERASE       0xA4
#define CMD_PROGRAM     0xA5
#define CMD_VERIFY      0xA6
#define CMD_READ_CONFIG 0xA7

#define CH592_CHIP_ID     0x92
#define CH59X_DEVICE_TYPE 0x22
#define CFG_MASK_ALL      0x1F

#define ISP_SEED         0xFF   /* semente enviada no ISP_KEY */
#define SECTOR_SIZE      1024   /* ERASE trabalha em setores de 1 KB */
#define MIN_ERASE_SECTOR 8      /* o bootloader apaga no minimo 8 setores */
#define CHUNK            56     /* bytes de firmware por PROGRAM/VERIFY */
#define CHUNK_TRIES      6      /* tentativas por bloco antes de desistir */
#define CHUNK_TIMEOUT_MS 3000   /* prazo para a resposta de um bloco */
#define MAX_DATA         64     /* maior resposta que esperamos */

struct isp_resp {
	uint8_t cmd;
	uint8_t status;
	uint16_t len;
	uint8_t data[MAX_DATA];
};

/* Diagnostico: bytes recebidos na uart1 desde o ultimo relatorio */
static uint32_t rx_count;
static uint8_t rx_last[16];

static int rx_poll(const struct device *uart, uint8_t *c)
{
	if (uart_poll_in(uart, c) != 0) {
		return -1;
	}
	rx_last[rx_count % sizeof(rx_last)] = *c;
	rx_count++;
	return 0;
}

static void rx_drain(const struct device *uart)
{
	uint8_t c;

	while (rx_poll(uart, &c) == 0) {
	}
}

static int rx_byte(const struct device *uart, uint8_t *c, int64_t deadline)
{
	while (rx_poll(uart, c) != 0) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/*
 * Um comando completo: todos os comandos do gravador passam por aqui.
 *
 * Pedido:   57 AB | cmd | len(LE16) | dados | soma(cmd..dados)
 * Resposta: 55 AA | cmd | status | len(LE16) | dados | soma(cmd..dados)
 *
 * Retorna 0 se chegou resposta valida (checksum e cmd conferem).
 */
static int isp_xfer(const struct device *uart, uint8_t cmd, const uint8_t *data,
		    uint16_t len, struct isp_resp *resp, uint32_t timeout_ms)
{
	uint8_t hdr[4] = { cmd, len & 0xFF, len >> 8 };
	uint8_t sum = 0;
	uint8_t c;
	int64_t deadline;

	rx_drain(uart);

	uart_poll_out(uart, 0x57);
	uart_poll_out(uart, 0xAB);
	for (int i = 0; i < 3; i++) {
		uart_poll_out(uart, hdr[i]);
		sum += hdr[i];
	}
	for (int i = 0; i < len; i++) {
		uart_poll_out(uart, data[i]);
		sum += data[i];
	}
	uart_poll_out(uart, sum);

	deadline = k_uptime_get() + timeout_ms;

	/* Procura o cabecalho 55 AA */
	do {
		if (rx_byte(uart, &c, deadline)) {
			return -ETIMEDOUT;
		}
		while (c == 0x55) {
			if (rx_byte(uart, &c, deadline)) {
				return -ETIMEDOUT;
			}
			if (c == 0xAA) {
				goto header;
			}
		}
	} while (1);

header:
	sum = 0;
	for (int i = 0; i < 4; i++) {
		if (rx_byte(uart, &hdr[i], deadline)) {
			return -ETIMEDOUT;
		}
		sum += hdr[i];
	}
	resp->cmd = hdr[0];
	resp->status = hdr[1];
	resp->len = hdr[2] | (hdr[3] << 8);
	if (resp->len > MAX_DATA) {
		return -EMSGSIZE;
	}
	for (int i = 0; i < resp->len; i++) {
		if (rx_byte(uart, &resp->data[i], deadline)) {
			return -ETIMEDOUT;
		}
		sum += resp->data[i];
	}
	if (rx_byte(uart, &c, deadline)) {
		return -ETIMEDOUT;
	}
	if (c != sum) {
		return -EBADMSG;
	}
	if (resp->cmd != cmd) {
		return -EPROTO;
	}
	return 0;
}

static void hexdump(const char *label, const uint8_t *buf, size_t len)
{
	printk("%s:", label);
	for (size_t i = 0; i < len; i++) {
		printk(" %02X", buf[i]);
	}
	printk("\n");
}

/* Diagnostico: mostra o que chegou na uart1 desde o ultimo relatorio */
static void rx_report(const struct device *uart)
{
	uint32_t n = MIN(rx_count, sizeof(rx_last));

	printk("uart1: %u bytes recebidos, erro=0x%x, ultimos:", rx_count,
	       uart_err_check(uart));
	for (uint32_t i = 0; i < n; i++) {
		printk(" %02X", rx_last[(rx_count - n + i) % sizeof(rx_last)]);
	}
	printk("\n");
	rx_count = 0;
}

/*
 * IDENTIFY leva o texto "MCU ISP & WCH.CN". Se o CH592 esta rodando a ponte,
 * esse texto o faz reiniciar no bootloader (reset magico, peripheral.c), e o
 * IDENTIFY seguinte ja e respondido pelo bootloader.
 */
int ch592_isp_connect(const struct device *uart, uint32_t timeout_ms)
{
	static const char magic[] = "MCU ISP & WCH.CN";
	uint8_t req[2 + sizeof(magic) - 1] = { CH592_CHIP_ID, CH59X_DEVICE_TYPE };
	struct isp_resp resp;
	int64_t deadline = k_uptime_get() + timeout_ms;

	memcpy(&req[2], magic, sizeof(magic) - 1);

	/* A janela do download sem botao e curta: insiste a cada ~20 ms */
	while (k_uptime_get() < deadline) {
		if (isp_xfer(uart, CMD_IDENTIFY, req, sizeof(req), &resp, 20) == 0 &&
		    resp.len >= 2) {
			printk("Bootloader encontrado: chip 0x%02X, tipo 0x%02X\n",
			       resp.data[0], resp.data[1]);
			if (resp.data[0] != CH592_CHIP_ID ||
			    resp.data[1] != CH59X_DEVICE_TYPE) {
				printk("Aviso: esperado chip 0x92 / tipo 0x22\n");
			}
			return 0;
		}
	}

	rx_report(uart);
	return -ETIMEDOUT;
}

/*
 * Prepara a sessao depois do IDENTIFY: le a config e o UID e envia a semente
 * da chave XOR. Usada no inicio e para ressincronizar depois de um erro.
 */
static int isp_session(const struct device *uart, uint8_t key[8], bool verbose)
{
	struct isp_resp resp;
	uint8_t req[30];
	uint8_t sum = 0;
	int ret;

	/* Config + versao do bootloader + UID */
	req[0] = CFG_MASK_ALL;
	req[1] = 0;
	ret = isp_xfer(uart, CMD_READ_CONFIG, req, 2, &resp, 500);
	if (ret || resp.len < 26) {
		printk("READ_CONFIG falhou (%d)\n", ret);
		return -EIO;
	}
	if (verbose) {
		hexdump("READ_CONFIG", resp.data, resp.len);
		printk("Bootloader V%x%x.%x%x, UID", resp.data[14], resp.data[15],
		       resp.data[16], resp.data[17]);
		for (int i = 18; i < 26; i++) {
			printk(" %02X", resp.data[i]);
		}
		printk("\n");
	}
	for (int i = 18; i < 26; i++) {
		sum += resp.data[i];
	}

	/*
	 * Chave XOR: o bootloader escolhe 7 bytes da semente e faz XOR com a soma
	 * do UID; o ultimo byte e o primeiro somado ao chip id. Com a semente
	 * uniforme, a escolha dos bytes nao importa. A semente e 0xFF e nao 0x00
	 * (wchisp): com 30 bytes 0x00 o bootloader nao respondia.
	 */
	memset(key, sum ^ ISP_SEED, 8);
	key[7] = key[0] + CH592_CHIP_ID;
	sum = 0;
	for (int i = 0; i < 8; i++) {
		sum += key[i];
	}

	memset(req, ISP_SEED, sizeof(req));
	for (int i = 0;; i++) {
		rx_count = 0;
		ret = isp_xfer(uart, CMD_ISP_KEY, req, 30, &resp, 1000);
		if (!ret && resp.len >= 1 && resp.data[0] == sum) {
			return 0;
		}
		printk("ISP_KEY falhou (%d, esperado 0x%02X)\n", ret, sum);
		if (!ret) {
			hexdump("resposta", resp.data, resp.len);
		}
		rx_report(uart);
		if (i == 2) {
			return -EIO;
		}
		k_msleep(100);
	}
}

/*
 * Depois de um frame perdido o bootloader fica dessincronizado e recusa os
 * blocos seguintes: fica em silencio para ele descartar o frame incompleto e
 * refaz IDENTIFY + sessao, sem apagar nada.
 */
static int isp_resync(const struct device *uart, uint8_t key[8])
{
	k_msleep(300);
	if (ch592_isp_connect(uart, 3000) != 0) {
		return -EIO;
	}
	return isp_session(uart, key, false);
}

/* Grava (PROGRAM) ou confere (VERIFY) o firmware em blocos de CHUNK bytes */
static int write_chunks(const struct device *uart, uint8_t cmd, const uint8_t *fw,
			size_t fw_len, uint8_t key[8])
{
	const char *name = cmd == CMD_PROGRAM ? "PROGRAM" : "VERIFY";
	/* O bootloader grava em palavras: o tamanho tem que ser multiplo de 8 */
	size_t len = ROUND_UP(fw_len, 8);
	uint8_t req[5 + CHUNK];
	struct isp_resp resp;
	size_t addr = 0;
	int mismatches = 0;

	while (addr < len) {
		size_t n = MIN(CHUNK, len - addr);

		/*
		 * Repete so este bloco. Regravar os mesmos dados e inofensivo (a
		 * flash so leva bits de 1 para 0) e o VERIFY confere tudo no final.
		 */
		for (int tries = 1, wrong = 0;; tries++) {
			int64_t t0, t;
			int ret;

			sys_put_le32(addr, req);
			req[4] = (uint8_t)k_cycle_get_32(); /* padding, qualquer valor */
			for (size_t i = 0; i < n; i++) {
				/* O fim do firmware e completado com 0xFF (flash apagada) */
				uint8_t b = addr + i < fw_len ? fw[addr + i] : 0xFF;

				/* Cada byte vai cifrado com a chave; o bootloader desfaz */
				req[5 + i] = b ^ key[i % 8];
			}

			t0 = k_uptime_get();
			ret = isp_xfer(uart, cmd, req, 5 + n, &resp, CHUNK_TIMEOUT_MS);
			t = k_uptime_get() - t0;
			if (t > 100) {
				printk("  %s em 0x%05zx levou %lld ms\n", name, addr, t);
			}
			if (!ret && (cmd == CMD_PROGRAM || (resp.len >= 1 && resp.data[0] == 0))) {
				break;
			}
			if (!ret) {
				/*
				 * Resposta valida dizendo que os dados nao conferem. Repete
				 * para separar erro na transmissao do VERIFY de dado gravado
				 * errado, e segue para contar quantos blocos estao errados.
				 */
				if (++wrong < 3) {
					continue;
				}
				printk("%s em 0x%05zx: dados diferentes (status 0x%02x)\n", name,
				       addr, resp.len ? resp.data[0] : 0xff);
				mismatches++;
				break;
			}
			printk("%s em 0x%05zx sem resposta (%d), tentativa %d\n", name, addr,
			       ret, tries);
			if (tries == CHUNK_TRIES) {
				return -EIO;
			}
			if (isp_resync(uart, key) != 0) {
				printk("Ressincronizacao falhou\n");
				return -EIO;
			}
		}
		addr += n;
		if (addr % (16 * SECTOR_SIZE) < CHUNK || addr == len) {
			printk("  %s %zu/%zu bytes\n",
			       cmd == CMD_PROGRAM ? "gravado" : "verificado", addr, len);
		}
	}

	if (mismatches) {
		printk("%s: %d de %zu blocos diferentes\n", name, mismatches,
		       DIV_ROUND_UP(len, CHUNK));
		return -EIO;
	}

	if (cmd == CMD_PROGRAM) {
		/* O bootloader exige um PROGRAM vazio no final */
		sys_put_le32(addr, req);
		req[4] = 0;
		if (isp_xfer(uart, CMD_PROGRAM, req, 5, &resp, 300)) {
			printk("PROGRAM final falhou\n");
			return -EIO;
		}
	}
	return 0;
}

/* A gravacao inteira; chamar depois de ch592_isp_connect() */
int ch592_isp_flash(const struct device *uart, const uint8_t *fw, size_t len)
{
	struct isp_resp resp;
	uint8_t req[4];
	uint8_t key[8];
	uint32_t sectors;
	int ret;

	ret = isp_session(uart, key, true);
	if (ret) {
		return ret;
	}

	sectors = MAX(DIV_ROUND_UP(len, SECTOR_SIZE), MIN_ERASE_SECTOR);
	printk("Apagando %u setores de 1 KB...\n", sectors);
	sys_put_le32(sectors, req);
	ret = isp_xfer(uart, CMD_ERASE, req, 4, &resp, 15000);
	if (ret) {
		printk("ERASE falhou (%d)\n", ret);
		return -EIO;
	}
	/* Margem para o bootloader terminar de apagar antes do primeiro bloco */
	k_msleep(200);

	printk("Gravando %zu bytes...\n", len);
	ret = write_chunks(uart, CMD_PROGRAM, fw, len, key);
	if (ret) {
		return ret;
	}

	printk("Verificando...\n");
	ret = write_chunks(uart, CMD_VERIFY, fw, len, key);
	if (ret) {
		return ret;
	}

	ch592_isp_end(uart);
	printk("Gravacao concluida, CH592 reiniciado\n");
	return 0;
}

void ch592_isp_end(const struct device *uart)
{
	uint8_t reason = 1;
	struct isp_resp resp;

	/* Sai do bootloader e roda o firmware gravado (pode nao haver resposta) */
	isp_xfer(uart, CMD_ISP_END, &reason, 1, &resp, 200);
}
