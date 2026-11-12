/**
 * @file radio.c
 * @brief RFM95W (SX1276) Meshtastic receiver driven directly over SPI.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * This version does not use the Zephyr LoRa driver and does not need DIO0:
 * the radio is configured register by register and a thread polls
 * RegIrqFlags over SPI to detect received packets. Only the SPI lines
 * (MOSI, MISO, SCK, NSS) are required. RESET is pulsed if wired, harmless
 * otherwise. Register map: Semtech SX1276/77/78/79 datasheet, LoRa mode.
 */
#include <ctype.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "meshtastic.h"
#include "radio.h"

LOG_MODULE_REGISTER(radio, LOG_LEVEL_INF);

#define LORA_NODE DT_ALIAS(lora0)

/* SX1276 registers (LoRa mode) */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_RX_HEADER_CNT_MSB    0x14
#define REG_RX_HEADER_CNT_LSB    0x15
#define REG_RX_PACKET_CNT_MSB    0x16
#define REG_RX_PACKET_CNT_LSB    0x17
#define REG_MODEM_STAT           0x18
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_MAX_PAYLOAD_LENGTH   0x23
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECT_OPTIMIZE      0x31
#define REG_INVERT_IQ            0x33
#define REG_HIGH_BW_OPTIMIZE_1   0x36
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_HIGH_BW_OPTIMIZE_2   0x3A
#define REG_INVERT_IQ_2          0x3B
#define REG_VERSION              0x42

#define MODE_LONG_RANGE  0x80
#define MODE_SLEEP       0x00
#define MODE_STDBY       0x01
#define MODE_RX_CONT     0x05

#define IRQ_RX_DONE          0x40
#define IRQ_PAYLOAD_CRC_ERR  0x20

#define MESHTASTIC_PREAMBLE  16
#define POLL_INTERVAL_MS     5

const struct mesh_region mesh_regions[] = {
	{"US", 902000000, 928000000},
	{"ANZ", 915000000, 928000000},
	{"BR_902", 902000000, 907500000},
};
const size_t mesh_region_count = ARRAY_SIZE(mesh_regions);

const struct mesh_preset mesh_presets[] = {
	{"LongFast", 250, 11, 5},
	{"LongMod", 125, 11, 8},
	{"LongSlow", 125, 12, 8},
	{"LongTurbo", 500, 11, 8},
	{"MediumFast", 250, 9, 5},
	{"MediumSlow", 250, 10, 5},
	{"ShortFast", 250, 7, 5},
	{"ShortSlow", 250, 8, 5},
	{"ShortTurbo", 500, 7, 5},
};
const size_t mesh_preset_count = ARRAY_SIZE(mesh_presets);

static const struct spi_dt_spec lora_spi =
	SPI_DT_SPEC_GET(LORA_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
static const struct gpio_dt_spec lora_reset = GPIO_DT_SPEC_GET_OR(LORA_NODE, reset_gpios, {0});

K_MSGQ_DEFINE(rx_queue, sizeof(struct radio_frame), 4, 4);
static K_MUTEX_DEFINE(radio_lock);

static struct radio_state state = {.sync_word = MESHTASTIC_SYNC_WORD};
static atomic_t rx_count;
static atomic_t crc_errors;
static bool radio_ready;

/* ------------------------------------------------------------ SPI access */

static int reg_read(uint8_t reg, uint8_t *value)
{
	uint8_t tx[2] = {reg & 0x7F, 0x00};
	uint8_t rx[2] = {0};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret = spi_transceive_dt(&lora_spi, &tx_set, &rx_set);

	*value = rx[1];
	return ret;
}

static int reg_write(uint8_t reg, uint8_t value)
{
	uint8_t tx[2] = {reg | 0x80, value};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	return spi_write_dt(&lora_spi, &tx_set);
}

/* Burst read of the FIFO: address byte, then len data bytes. */
static int fifo_read(uint8_t *data, size_t len)
{
	static uint8_t tx[257];
	static uint8_t rx[257];
	const struct spi_buf tx_buf = {.buf = tx, .len = len + 1};
	const struct spi_buf rx_buf = {.buf = rx, .len = len + 1};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;

	memset(tx, 0, len + 1);
	tx[0] = REG_FIFO;
	ret = spi_transceive_dt(&lora_spi, &tx_set, &rx_set);
	memcpy(data, &rx[1], len);
	return ret;
}

/* -------------------------------------------------------------- lookups */

static bool name_equals(const char *a, const char *b)
{
	while (*a && *b) {
		if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) {
			return false;
		}
	}
	return *a == *b;
}

const struct mesh_region *radio_find_region(const char *name)
{
	for (size_t i = 0; i < mesh_region_count; i++) {
		if (name_equals(name, mesh_regions[i].name)) {
			return &mesh_regions[i];
		}
	}
	return NULL;
}

const struct mesh_preset *radio_find_preset(const char *name)
{
	for (size_t i = 0; i < mesh_preset_count; i++) {
		if (name_equals(name, mesh_presets[i].name)) {
			return &mesh_presets[i];
		}
	}
	return NULL;
}

static uint8_t bw_code(uint16_t khz)
{
	switch (khz) {
	case 125: return 0x7;
	case 500: return 0x9;
	default: return 0x8; /* 250 kHz */
	}
}

/* ------------------------------------------------------- configuration */

static int configure(const struct mesh_region *region, const struct mesh_preset *preset)
{
	uint32_t bw_hz = (uint32_t)preset->bw_khz * 1000U;
	uint32_t symbol_us = (1U << preset->sf) * 1000U / preset->bw_khz;
	uint64_t frf;
	int ret = 0;

	/* Frequency slot (RadioInterface::applyModemConfig): the default channel
	 * name is the preset name, hashed with djb2.
	 */
	state.region = region;
	state.preset = preset;
	state.num_slots = (region->end_hz - region->start_hz) / bw_hz;
	state.slot = mesh_djb2(preset->name) % state.num_slots;
	state.freq_hz = region->start_hz + bw_hz / 2U + state.slot * bw_hz;
	state.channel_hash = mesh_channel_hash(preset->name, mesh_default_key,
					       sizeof(mesh_default_key));
	frf = ((uint64_t)state.freq_hz << 19) / 32000000U;

	/* LoRa mode can only be selected in sleep. */
	ret |= reg_write(REG_OP_MODE, MODE_SLEEP);
	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);

	ret |= reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
	ret |= reg_write(REG_FRF_MID, (uint8_t)(frf >> 8));
	ret |= reg_write(REG_FRF_LSB, (uint8_t)frf);

	ret |= reg_write(REG_LNA, 0x23); /* max gain, LNA boost for the HF port */

	/* Bandwidth, coding rate, explicit header */
	ret |= reg_write(REG_MODEM_CONFIG_1, (bw_code(preset->bw_khz) << 4) |
					     ((preset->cr - 4) << 1));
	/* Spreading factor, payload CRC on */
	ret |= reg_write(REG_MODEM_CONFIG_2, (preset->sf << 4) | 0x04);
	/* AGC on; low data rate optimize when a symbol lasts more than 16 ms */
	ret |= reg_write(REG_MODEM_CONFIG_3, 0x04 | (symbol_us > 16000U ? 0x08 : 0x00));

	ret |= reg_write(REG_PREAMBLE_MSB, MESHTASTIC_PREAMBLE >> 8);
	ret |= reg_write(REG_PREAMBLE_LSB, MESHTASTIC_PREAMBLE & 0xFF);
	ret |= reg_write(REG_SYNC_WORD, state.sync_word);

	/* SF7..SF12 detection settings, normal IQ */
	ret |= reg_write(REG_DETECT_OPTIMIZE, 0xC3);
	ret |= reg_write(REG_DETECTION_THRESHOLD, 0x0A);
	ret |= reg_write(REG_INVERT_IQ, 0x27);
	ret |= reg_write(REG_INVERT_IQ_2, 0x1D);

	/* Errata 2.1: sensitivity optimization with a 500 kHz bandwidth */
	if (preset->bw_khz == 500) {
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_1, 0x02);
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_2, 0x64);
	} else {
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_1, 0x03);
	}

	ret |= reg_write(REG_MAX_PAYLOAD_LENGTH, 0xFF);
	ret |= reg_write(REG_FIFO_RX_BASE_ADDR, 0x00);
	ret |= reg_write(REG_FIFO_ADDR_PTR, 0x00);
	ret |= reg_write(REG_IRQ_FLAGS_MASK, 0x00);
	ret |= reg_write(REG_IRQ_FLAGS, 0xFF);

	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT);
	return ret ? -EIO : 0;
}

int radio_set(const struct mesh_region *region, const struct mesh_preset *preset)
{
	int ret;

	k_mutex_lock(&radio_lock, K_FOREVER);
	ret = configure(region, preset);
	k_mutex_unlock(&radio_lock);

	if (ret < 0) {
		LOG_ERR("Radio configuration failed: %d", ret);
		return ret;
	}

	LOG_INF("Listening: %s %s, %u.%03u MHz (slot %u/%u), SF%u BW%u CR4/%u, sync 0x%02x%s",
		region->name, preset->name, state.freq_hz / 1000000U,
		(state.freq_hz / 1000U) % 1000U, state.slot + 1, state.num_slots, preset->sf,
		preset->bw_khz, preset->cr, state.sync_word,
		state.sync_word == MESHTASTIC_SYNC_WORD ? "" : " (raw test mode)");
	return 0;
}

int radio_init(const struct mesh_region *region, const struct mesh_preset *preset)
{
	uint8_t version = 0;

	if (!spi_is_ready_dt(&lora_spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Hardware reset, if the RESET line is wired (harmless if it is not). */
	if (lora_reset.port != NULL && gpio_is_ready_dt(&lora_reset)) {
		gpio_pin_configure_dt(&lora_reset, GPIO_OUTPUT_ACTIVE);
		k_msleep(1);
		gpio_pin_set_dt(&lora_reset, 0);
		k_msleep(10);
	}

	if (reg_read(REG_VERSION, &version) < 0 || (version != 0x11 && version != 0x12)) {
		LOG_ERR("SX1276 not found (RegVersion = 0x%02x). Check the SPI wiring.", version);
		return -ENODEV;
	}
	LOG_INF("SX1276 found (RegVersion = 0x%02x), polling mode (no DIO0)", version);

	radio_ready = true;
	return radio_set(region, preset);
}

/* ----------------------------------------------------- polling thread */

/* Packet RSSI, HF port (datasheet 5.5.5). */
static int16_t packet_rssi(uint8_t raw_rssi, int8_t snr_quarter_db)
{
	int16_t rssi = -157 + raw_rssi;

	if (snr_quarter_db < 0) {
		rssi += snr_quarter_db / 4;
	}
	return rssi;
}

static void poll_thread(void *a, void *b, void *c)
{
	static struct radio_frame frame;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		uint8_t flags = 0, nb = 0, addr = 0, raw_snr = 0, raw_rssi = 0;
		bool got_frame = false;

		k_msleep(POLL_INTERVAL_MS);
		if (!radio_ready) {
			continue;
		}

		k_mutex_lock(&radio_lock, K_FOREVER);
		if (reg_read(REG_IRQ_FLAGS, &flags) == 0 && (flags & IRQ_RX_DONE)) {
			if (flags & IRQ_PAYLOAD_CRC_ERR) {
				atomic_inc(&crc_errors);
			} else {
				reg_read(REG_RX_NB_BYTES, &nb);
				reg_read(REG_FIFO_RX_CURRENT_ADDR, &addr);
				reg_read(REG_PKT_SNR_VALUE, &raw_snr);
				reg_read(REG_PKT_RSSI_VALUE, &raw_rssi);
				reg_write(REG_FIFO_ADDR_PTR, addr);
				fifo_read(frame.data, nb);
				frame.len = nb;
				frame.snr = (int8_t)raw_snr / 4;
				frame.rssi = packet_rssi(raw_rssi, (int8_t)raw_snr);
				got_frame = true;
			}
			reg_write(REG_IRQ_FLAGS, 0xFF);
		}
		k_mutex_unlock(&radio_lock);

		if (got_frame) {
			atomic_inc(&rx_count);
			if (k_msgq_put(&rx_queue, &frame, K_NO_WAIT) != 0) {
				LOG_WRN("RX queue full, frame dropped");
			}
		}
	}
}

K_THREAD_DEFINE(radio_poll_tid, 2048, poll_thread, NULL, NULL, NULL, 6, 0, 0);

/* -------------------------------------------------------------- queries */

int radio_set_sync_word(uint8_t sync_word)
{
	state.sync_word = sync_word;
	return radio_set(state.region, state.preset);
}

const struct radio_state *radio_get(void)
{
	return &state;
}

int radio_get_frame(struct radio_frame *frame, int32_t timeout_ms)
{
	return k_msgq_get(&rx_queue, frame, timeout_ms < 0 ? K_FOREVER : K_MSEC(timeout_ms));
}

uint32_t radio_rx_count(void)
{
	return (uint32_t)atomic_get(&rx_count);
}

uint32_t radio_crc_errors(void)
{
	return (uint32_t)atomic_get(&crc_errors);
}

/* ---------------------------------------------------------- diagnostics */

int radio_read_diag(struct radio_diag *d)
{
	uint8_t f[3], h[2], p[2], pre[2], rssi = 0;
	int ret = 0;

	k_mutex_lock(&radio_lock, K_FOREVER);
	ret |= reg_read(REG_VERSION, &d->version);
	ret |= reg_read(REG_OP_MODE, &d->op_mode);
	ret |= reg_read(REG_FRF_MSB, &f[0]);
	ret |= reg_read(REG_FRF_MID, &f[1]);
	ret |= reg_read(REG_FRF_LSB, &f[2]);
	ret |= reg_read(REG_MODEM_CONFIG_1, &d->modem_config_1);
	ret |= reg_read(REG_MODEM_CONFIG_2, &d->modem_config_2);
	ret |= reg_read(REG_MODEM_CONFIG_3, &d->modem_config_3);
	ret |= reg_read(REG_SYNC_WORD, &d->sync_word);
	ret |= reg_read(REG_PREAMBLE_MSB, &pre[0]);
	ret |= reg_read(REG_PREAMBLE_LSB, &pre[1]);
	ret |= reg_read(REG_MODEM_STAT, &d->modem_stat);
	ret |= reg_read(REG_RX_HEADER_CNT_MSB, &h[0]);
	ret |= reg_read(REG_RX_HEADER_CNT_LSB, &h[1]);
	ret |= reg_read(REG_RX_PACKET_CNT_MSB, &p[0]);
	ret |= reg_read(REG_RX_PACKET_CNT_LSB, &p[1]);
	ret |= reg_read(REG_RSSI_VALUE, &rssi);
	k_mutex_unlock(&radio_lock);

	d->frf = ((uint32_t)f[0] << 16) | ((uint32_t)f[1] << 8) | f[2];
	d->preamble = ((uint16_t)pre[0] << 8) | pre[1];
	d->valid_headers = ((uint16_t)h[0] << 8) | h[1];
	d->valid_packets = ((uint16_t)p[0] << 8) | p[1];
	d->rssi_now = -157 + rssi; /* HF port, datasheet 5.5.5 */
	return ret ? -EIO : 0;
}

void radio_expected_diag(struct radio_diag *d)
{
	const struct mesh_preset *p = state.preset;
	uint32_t symbol_us = (1U << p->sf) * 1000U / p->bw_khz;

	memset(d, 0, sizeof(*d));
	d->op_mode = MODE_LONG_RANGE | MODE_RX_CONT;
	d->frf = (uint32_t)(((uint64_t)state.freq_hz << 19) / 32000000U);
	d->modem_config_1 = (bw_code(p->bw_khz) << 4) | ((p->cr - 4) << 1);
	d->modem_config_2 = (p->sf << 4) | 0x04;
	d->modem_config_3 = 0x04 | (symbol_us > 16000U ? 0x08 : 0x00);
	d->sync_word = state.sync_word;
	d->preamble = MESHTASTIC_PREAMBLE;
}
