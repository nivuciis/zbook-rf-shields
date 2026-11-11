/**
 * @file main.c
 *
 * @brief SPI check for RFM95 module with Zephyr RTOS on the ZBook board.
 *
 * Reads the SX1276 RegVersion register (0x42) in batches and logs how many
 * reads returned a valid silicon revision: 0x11 (HopeRF datasheet) or 0x12
 * (Semtech production silicon V1b). Nothing is transmitted over the air.
 *
 * @author Vinicius RM Carvalho
 * @date 2026-11-11
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define REG_VERSION 0x42
#define BATCH_SIZE  20000

static const struct spi_dt_spec rfm95 =
	SPI_DT_SPEC_GET(DT_NODELABEL(rfm95), SPI_WORD_SET(8) | SPI_TRANSFER_MSB);

/**
 * @brief Read one SX1276 register over SPI.
 *
 * Byte 1 sends the address with bit 7 cleared (read access); byte 2 is a
 * dummy byte that clocks the register value out on MISO.
 *
 * @param reg   Register address.
 * @param value Where the register value is stored.
 * @return 0 on success, negative errno from the SPI driver otherwise.
 */
static int rfm95_read(uint8_t reg, uint8_t *value)
{
	uint8_t tx[2] = {reg & 0x7F, 0x00};
	uint8_t rx[2] = {0};

	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	int ret = spi_transceive_dt(&rfm95, &tx_set, &rx_set);

	*value = rx[1];
	return ret;
}

int main(void)
{
	if (!spi_is_ready_dt(&rfm95)) {
		LOG_ERR("SPI1 IS NOT READY");
		return 0;
	}

	while (1) {
		uint8_t version = 0;
		uint32_t ok = 0;

		for (int i = 0; i < BATCH_SIZE; i++) {
			if (rfm95_read(REG_VERSION, &version) == 0 &&
			    (version == 0x11 || version == 0x12)) {
				ok++;
			}
		}

		LOG_INF("OK %5u / %u   (last value 0x%02x)", ok, BATCH_SIZE, version);
	}

	return 0;
}