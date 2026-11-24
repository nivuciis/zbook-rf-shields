#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include "ble_uart.h"
#include "ch592_isp.h"

/*
 * Bluetooth da zbook pelo kit CH592X-EVT-R1, ligado na uart1 (header H2):
 *
 *   zbook TX  (GPIO20) -> P4 UART1_RX (PA8)
 *   zbook RX  (GPIO21) <- P4 UART1_TX (PA9)
 *   zbook GND          -- P4 GND
 *
 * Ao ligar, a zbook grava o firmware embutido (CH592_HEX, CMakeLists.txt) no
 * CH592 se ele mudou desde a ultima gravacao, ou se BUTTON0 estiver apertado.
 * Depois usa o CH592 como ponte BLE <-> UART (ch592/ble_uart): o que o
 * celular envia e impresso no console (uart0) e cada linha digitada no
 * console e enviada ao celular.
 *
 * Mapa do arquivo:
 *   ch592_fw[]       firmware do CH592 embutido (array gerado no build)
 *   ch592_update()   decide se grava: CRC do array x CRC salvo no LittleFS
 *   ch592_program()  grava pelo protocolo ISP (ch592_isp.c)
 *   console_to_ble() thread da volta: console -> celular
 *   ble_bridge()     ida: celular -> console (regra dos 50 ms)
 *   main()           update -> liga interrupcao -> ponte
 */

/*
 * O CMake gera ch592_fw.inc (texto "0x13, 0x05, ...") a partir do .bin; o
 * include entre as chaves vira um array constante na flash da zbook.
 */
static const uint8_t ch592_fw[] = {
#include "ch592_fw.inc"
};

/* CRC do ultimo firmware gravado no CH592 */
#define FW_INFO_PATH "/lfs/ch592.crc"

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

/* O firmware ble_uart contem o texto que dispara o reset para o bootloader */
static bool ch592_fw_is_ble_uart(void)
{
	static const char magic[] = "MCU ISP & WCH.CN";

	for (size_t i = 0; i + sizeof(magic) - 1 <= sizeof(ch592_fw); i++) {
		if (memcmp(&ch592_fw[i], magic, sizeof(magic) - 1) == 0) {
			return true;
		}
	}
	return false;
}

/* O que fica salvo em /lfs/ch592.crc: so o CRC e o tamanho, nao o firmware */
struct fw_info {
	uint32_t crc;
	uint32_t len;
};

/* Le o CRC da ultima gravacao; false se o arquivo nao existe (primeira vez) */
static bool fw_info_read(struct fw_info *info)
{
	struct fs_dirent entry;
	struct fs_file_t file;
	ssize_t n;

	if (fs_stat(FW_INFO_PATH, &entry) != 0) {
		return false;
	}
	fs_file_t_init(&file);
	if (fs_open(&file, FW_INFO_PATH, FS_O_READ) != 0) {
		return false;
	}
	n = fs_read(&file, info, sizeof(*info));
	fs_close(&file);
	return n == sizeof(*info);
}

/* Salva o CRC do firmware que acabou de ser gravado */
static void fw_info_write(const struct fw_info *info)
{
	struct fs_file_t file;

	fs_file_t_init(&file);
	if (fs_open(&file, FW_INFO_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC) != 0) {
		printk("Aviso: nao foi possivel salvar %s\n", FW_INFO_PATH);
		return;
	}
	fs_write(&file, info, sizeof(*info));
	fs_close(&file);
}

/* Acha o bootloader e grava; insiste ate conseguir */
static void ch592_program(void)
{
	printk("Gravando o CH592 (%zu bytes) pela uart1...\n", sizeof(ch592_fw));
	printk("Se o firmware atual do CH592 nao for o ble_uart, desligue o kit, "
	       "tire o fio TX da zbook, ligue segurando Download e recoloque o fio.\n");

	/* Depois de uma falha o CH592 continua no bootloader: tenta de novo */
	for (int tries = 1;; tries++) {
		while (ch592_isp_connect(uart, 10000) != 0) {
			printk("Nenhum bootloader respondeu, tentando de novo...\n");
		}
		if (ch592_isp_flash(uart, ch592_fw, sizeof(ch592_fw)) == 0) {
			return;
		}
		printk("Gravacao FALHOU (tentativa %d), repetindo...\n", tries);
		k_msleep(500);
	}
}

/*
 * Autoteste da regravacao sem botao: o firmware ble_uart reinicia no
 * bootloader quando recebe o IDENTIFY. Se o bootloader responder, o firmware
 * esta rodando e a proxima atualizacao sera automatica.
 */
static void ch592_selftest(void)
{
	k_msleep(1500); /* tempo para o firmware BLE iniciar */
	if (ch592_isp_connect(uart, 3000) == 0) {
		printk("Autoteste OK: o CH592 volta ao bootloader pela uart1, "
		       "as proximas gravacoes serao automaticas\n");
		ch592_isp_end(uart);
		k_msleep(500);
	} else {
		printk("Autoteste FALHOU: o firmware BLE nao respondeu ao pedido de "
		       "regravacao (ele esta rodando?)\n");
	}
}

/*
 * Roda uma vez por boot. Nao grava se o CRC do array for igual ao salvo E o
 * BUTTON0 estiver solto; senao grava, salva o CRC novo e faz o autoteste.
 * O firmware nao e recompilado aqui: isso acontece no PC (west build).
 */
static void ch592_update(void)
{
	struct fw_info now = {
		.crc = crc32_ieee(ch592_fw, sizeof(ch592_fw)),
		.len = sizeof(ch592_fw),
	};
	struct fw_info last;
	bool forced = gpio_pin_get_dt(&button) > 0;

	if (!forced && fw_info_read(&last) && last.crc == now.crc && last.len == now.len) {
		printk("CH592 ja esta com o firmware atual (crc 0x%08x)\n", now.crc);
		return;
	}

	printk("CH592: firmware novo (crc 0x%08x)%s\n", now.crc,
	       forced ? ", gravacao forcada por BUTTON0" : "");
	ch592_program();
	fw_info_write(&now);
	if (ch592_fw_is_ble_uart()) {
		ch592_selftest();
	}
}

/*
 * Volta: zbook -> celular. Cada linha digitada no console (uart0) e enviada
 * ao celular pela ponte. A thread so comeca depois que a gravacao do CH592
 * terminou (ble_ready), para nao misturar texto com o protocolo ISP.
 * O semaforo e esperado uma vez so, no inicio; depois as linhas passam direto.
 */
static K_SEM_DEFINE(ble_ready, 0, 1);

static void console_to_ble(void *p1, void *p2, void *p3)
{
	static const char magic[] = "MCU ISP & WCH.CN";

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_take(&ble_ready, K_FOREVER);
	console_getline_init();

	while (1) {
		char *line = console_getline();

		if (line == NULL || line[0] == '\0') {
			continue;
		}
		/* Esse texto faria o CH592 reiniciar no bootloader */
		if (strstr(line, magic) != NULL) {
			printk("Nao enviado: o texto reinicia o CH592 no bootloader\n");
			continue;
		}
		ble_uart_send(line, strlen(line));
		ble_uart_send("\r\n", 2);
		printk("%s\n", line);
	}
}

/* Thread criada no boot: pilha de 1 KB, prioridade 7, comeca ja (e para no semaforo) */
K_THREAD_DEFINE(console_tid, 1024, console_to_ble, NULL, NULL, NULL, 7, 0, 0);

/*
 * Ida: celular -> zbook. Cada mensagem recebida e impressa no console. Uma
 * mensagem termina em fim de linha ou depois de 50 ms sem bytes novos (apps
 * como o nRF Connect nao mandam fim de linha).
 */
static void print_msg(char *line, size_t len)
{
	line[len] = '\0';
	printk("%s\n", line);
}

/*
 * Laco eterno da thread main: tira bytes do ring buffer, junta em linha e
 * imprime. Libera a thread da volta (k_sem_give) logo no comeco.
 */
static void ble_bridge(void)
{
	char line[128];
	size_t len = 0;
	int64_t last_rx = 0;
	uint8_t c;

	printk("BLE pronto: conecte no dispositivo \"ch592_ble_uart\" (ex.: nRF Connect), "
	       "ative as notificacoes de 0xFFF1 e escreva em 0xFFF2.\n"
	       "O que o celular enviar aparece aqui; digite uma linha e Enter para "
	       "enviar ao celular.\n");
	k_sem_give(&ble_ready);

	while (1) {
		while (ble_uart_read(&c, 1) == 1) {
			last_rx = k_uptime_get();
			if (c == '\r' || c == '\n' || len == sizeof(line) - 1) {
				if (len > 0) {
					print_msg(line, len);
					len = 0;
				}
			} else {
				line[len++] = c;
			}
		}
		if (len > 0 && k_uptime_get() - last_rx > 50) {
			print_msg(line, len);
			len = 0;
		}

		k_msleep(10);
	}
}

/* A ordem importa: primeiro grava (uart1 em polling), depois a ponte (interrupcao) */
int main(void)
{
	if (!device_is_ready(uart) || !gpio_is_ready_dt(&button)) {
		printk("uart1 ou BUTTON0 nao estao prontos\n");
		return 0;
	}
	gpio_pin_configure_dt(&button, GPIO_INPUT);

	ch592_update();

	if (ble_uart_init(uart) != 0) {
		printk("Falha ao ligar a recepcao da uart1\n");
		return 0;
	}
	ble_bridge();
	return 0;
}
