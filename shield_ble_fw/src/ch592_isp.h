#ifndef CH592_ISP_H_
#define CH592_ISP_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

/*
 * Gravador do CH592 pelo bootloader ISP serial da WCH (UART1 do CH592:
 * PA8 = RX, PA9 = TX, 115200 8N1). Protocolo baseado no wchisp
 * (github.com/ch32-rs/wchisp).
 */

/* Envia IDENTIFY repetidamente ate o bootloader responder ou estourar timeout_ms */
int ch592_isp_connect(const struct device *uart, uint32_t timeout_ms);

/* Le config/UID, apaga, grava, verifica e reinicia o CH592 no firmware novo */
int ch592_isp_flash(const struct device *uart, const uint8_t *fw, size_t len);

/* Sai do bootloader e roda o firmware que esta na flash do CH592 */
void ch592_isp_end(const struct device *uart);

#endif /* CH592_ISP_H_ */
