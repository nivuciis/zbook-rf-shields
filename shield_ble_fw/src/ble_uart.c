/*
 * Lado da zbook na ponte: a uart1 depois da gravacao.
 *
 *   Recebe (CH592 -> zbook): interrupcao -> ring buffer -> ble_uart_read()
 *   Envia  (zbook -> CH592): ble_uart_send(), byte a byte por polling
 *
 * Usa so a API generica de UART do Zephyr; o driver por baixo e o PL011.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "ble_uart.h"

/*
 * Buffer circular entre a interrupcao (produtor) e a thread main (consumidor).
 * Se encher, ring_buf_put recusa os bytes novos (os antigos ficam).
 */
RING_BUF_DECLARE(rx_buf, 1024);

static const struct device *ble_dev;

/*
 * Roda na interrupcao da uart1: so copia o FIFO de hardware (32 bytes) para o
 * ring buffer e sai. Nada de printk ou espera aqui.
 */
static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t chunk[32];
	int n;

	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		n = uart_fifo_read(dev, chunk, sizeof(chunk));
		if (n <= 0) {
			break;
		}
		ring_buf_put(&rx_buf, chunk, n);
	}
}

/* Chamada uma vez, depois da gravacao: troca o polling do ISP por interrupcao */
int ble_uart_init(const struct device *uart)
{
	uint8_t c;
	int ret;

	ble_dev = uart;

	/* Descarta o que sobrou da gravacao */
	while (uart_poll_in(uart, &c) == 0) {
	}

	ret = uart_irq_callback_user_data_set(uart, uart_isr, NULL);
	if (ret) {
		return ret;
	}
	uart_irq_rx_enable(uart);
	return 0;
}

/* Envio simples por polling: a 115200 e com mensagens curtas e suficiente */
void ble_uart_send(const void *data, size_t len)
{
	const uint8_t *p = data;

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(ble_dev, p[i]);
	}
}

/* Lado consumidor: a thread main tira do ring buffer no ritmo dela */
size_t ble_uart_read(void *buf, size_t len)
{
	return ring_buf_get(&rx_buf, buf, len);
}
