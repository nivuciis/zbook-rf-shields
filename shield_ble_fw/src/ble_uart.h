#ifndef BLE_UART_H_
#define BLE_UART_H_

#include <stddef.h>
#include <zephyr/device.h>

/*
 * Bluetooth da zbook pelo CH592: o firmware ch592/ble_uart e uma ponte
 * transparente. O que a zbook escreve na uart1 vai para o celular (notify) e o
 * que o celular escreve chega na uart1.
 *
 * Nao envie o texto "MCU ISP & WCH.CN": ele faz o CH592 reiniciar no
 * bootloader para ser regravado.
 */

/* Liga a recepcao por interrupcao (depois de gravar o CH592) */
int ble_uart_init(const struct device *uart);

/* Envia dados para o celular conectado */
void ble_uart_send(const void *data, size_t len);

/* Copia ate len bytes recebidos do celular; retorna quantos copiou */
size_t ble_uart_read(void *buf, size_t len);

#endif /* BLE_UART_H_ */
