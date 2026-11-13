/**
 * @file radio.c
 * @brief RFM95W (SX1276) LoRa transceiver for Meshtastic, driven over SPI.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * The chip is configured register by register (Semtech SX1276 datasheet,
 * LoRa mode). DIO0 is mapped to RxDone in receive and TxDone in transmit and,
 * when it is wired, wakes the mesh thread through a GPIO interrupt. The same
 * events are always confirmed by reading RegIrqFlags over SPI, so the radio
 * keeps working (by polling) if DIO0 is missing or on the wrong pin.
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
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_STAT           0x18
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MAX_PAYLOAD_LENGTH   0x23
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECT_OPTIMIZE      0x31
#define REG_INVERT_IQ            0x33
#define REG_HIGH_BW_OPTIMIZE_1   0x36
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_HIGH_BW_OPTIMIZE_2   0x3A
#define REG_INVERT_IQ_2          0x3B
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

#define MODE_LONG_RANGE  0x80
#define MODE_SLEEP       0x00
#define MODE_STDBY       0x01
#define MODE_TX          0x03
#define MODE_RX_CONT     0x05

#define IRQ_RX_DONE          0x40
#define IRQ_PAYLOAD_CRC_ERR  0x20
#define IRQ_TX_DONE          0x08

#define DIO0_RX_DONE  0x00  /* RegDioMapping1 bits 7:6 = 00 */
#define DIO0_TX_DONE  0x40  /* RegDioMapping1 bits 7:6 = 01 */

#define MESHTASTIC_SYNC_WORD 0x2B
#define MESHTASTIC_PREAMBLE  16

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
static const struct gpio_dt_spec lora_dio0 = GPIO_DT_SPEC_GET_BY_IDX_OR(LORA_NODE, dio_gpios, 0, {0});

static K_MUTEX_DEFINE(radio_lock);
static K_SEM_DEFINE(irq_sem, 0, 1);
static struct gpio_callback dio0_cb;
static atomic_t dio0_edge;

static struct radio_state state = {.tx_power_dbm = 17};
static struct radio_stats stats;

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

static uint8_t burst_tx[257];
static uint8_t burst_rx[257];

static int fifo_read(uint8_t *data, size_t len)
{
	const struct spi_buf tx_buf = {.buf = burst_tx, .len = len + 1};
	const struct spi_buf rx_buf = {.buf = burst_rx, .len = len + 1};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;

	memset(burst_tx, 0, len + 1);
	burst_tx[0] = REG_FIFO;
	ret = spi_transceive_dt(&lora_spi, &tx_set, &rx_set);
	memcpy(data, &burst_rx[1], len);
	return ret;
}

static int fifo_write(const uint8_t *data, size_t len)
{
	const struct spi_buf tx_buf = {.buf = burst_tx, .len = len + 1};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	burst_tx[0] = REG_FIFO | 0x80;
	memcpy(&burst_tx[1], data, len);
	return spi_write_dt(&lora_spi, &tx_set);
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

static uint32_t symbol_us(void)
{
	return (1U << state.preset->sf) * 1000U / state.preset->bw_khz;
}

/* ------------------------------------------------------- configuration */

static uint8_t pa_config_value(void)
{
	/* PA_BOOST output (RFM95W): Pout = 2 + OutputPower, up to 17 dBm. */
	return 0x80 | 0x70 | (uint8_t)(state.tx_power_dbm - 2);
}

static int start_rx(void)
{
	int ret = 0;

	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
	ret |= reg_write(REG_DIO_MAPPING_1, DIO0_RX_DONE);
	ret |= reg_write(REG_FIFO_RX_BASE_ADDR, 0x00);
	ret |= reg_write(REG_FIFO_ADDR_PTR, 0x00);
	ret |= reg_write(REG_IRQ_FLAGS, 0xFF);
	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_RX_CONT);
	return ret;
}

static int configure(const struct mesh_region *region, const struct mesh_preset *preset)
{
	uint32_t bw_hz = (uint32_t)preset->bw_khz * 1000U;
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

	ret |= reg_write(REG_PA_CONFIG, pa_config_value());
	ret |= reg_write(REG_LNA, 0x23); /* max gain, LNA boost for the HF port */

	ret |= reg_write(REG_MODEM_CONFIG_1, (bw_code(preset->bw_khz) << 4) |
					     ((preset->cr - 4) << 1));     /* explicit header */
	ret |= reg_write(REG_MODEM_CONFIG_2, (preset->sf << 4) | 0x04); /* CRC on */
	/* AGC on; low data rate optimize when a symbol lasts more than 16 ms */
	ret |= reg_write(REG_MODEM_CONFIG_3, 0x04 | (symbol_us() > 16000U ? 0x08 : 0x00));

	ret |= reg_write(REG_PREAMBLE_MSB, MESHTASTIC_PREAMBLE >> 8);
	ret |= reg_write(REG_PREAMBLE_LSB, MESHTASTIC_PREAMBLE & 0xFF);
	ret |= reg_write(REG_SYNC_WORD, MESHTASTIC_SYNC_WORD);

	ret |= reg_write(REG_DETECT_OPTIMIZE, 0xC3);     /* SF7..SF12 */
	ret |= reg_write(REG_DETECTION_THRESHOLD, 0x0A);
	ret |= reg_write(REG_INVERT_IQ, 0x27);           /* normal IQ */
	ret |= reg_write(REG_INVERT_IQ_2, 0x1D);

	/* Errata 2.1: sensitivity optimization with a 500 kHz bandwidth */
	if (preset->bw_khz == 500) {
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_1, 0x02);
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_2, 0x64);
	} else {
		ret |= reg_write(REG_HIGH_BW_OPTIMIZE_1, 0x03);
	}

	ret |= reg_write(REG_MAX_PAYLOAD_LENGTH, 0xFF);
	ret |= reg_write(REG_FIFO_TX_BASE_ADDR, 0x00);
	ret |= reg_write(REG_IRQ_FLAGS_MASK, 0x00);

	ret |= start_rx();
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

	LOG_INF("Radio: %s %s, %u.%03u MHz (slot %u/%u), SF%u BW%u CR4/%u, %u dBm",
		region->name, preset->name, state.freq_hz / 1000000U,
		(state.freq_hz / 1000U) % 1000U, state.slot + 1, state.num_slots, preset->sf,
		preset->bw_khz, preset->cr, state.tx_power_dbm);
	return 0;
}

int radio_set_tx_power(uint8_t dbm)
{
	if (dbm < 2 || dbm > 17) {
		return -EINVAL;
	}
	k_mutex_lock(&radio_lock, K_FOREVER);
	state.tx_power_dbm = dbm;
	reg_write(REG_PA_CONFIG, pa_config_value());
	k_mutex_unlock(&radio_lock);
	return 0;
}

static void dio0_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	atomic_set(&dio0_edge, 1);
	k_sem_give(&irq_sem);
}

static void setup_dio0(void)
{
	if (lora_dio0.port == NULL || !gpio_is_ready_dt(&lora_dio0)) {
		LOG_WRN("DIO0 not in the devicetree: polling only");
		return;
	}
	gpio_pin_configure_dt(&lora_dio0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&lora_dio0, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&dio0_cb, dio0_isr, BIT(lora_dio0.pin));
	gpio_add_callback(lora_dio0.port, &dio0_cb);
	stats.dio0_configured = true;
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
	LOG_INF("SX1276 found (RegVersion = 0x%02x)", version);

	setup_dio0();
	return radio_set(region, preset);
}

/* ------------------------------------------------------------- RX / TX */

void radio_wait_event(int32_t timeout_ms)
{
	k_sem_take(&irq_sem, K_MSEC(timeout_ms));
}

/* Keep track of whether DIO0 announced the event we just found by polling. */
static void account_dio0(void)
{
	if (atomic_cas(&dio0_edge, 1, 0)) {
		stats.dio0_hits++;
	} else if (stats.dio0_configured) {
		stats.dio0_misses++;
		if (stats.dio0_misses == 3 && stats.dio0_hits == 0) {
			LOG_WRN("DIO0 never fired: check the IO46 wire. Polling keeps working.");
		}
	}
}

/* Packet RSSI, HF port (datasheet 5.5.5). */
static int16_t packet_rssi(uint8_t raw_rssi, int8_t snr_quarter_db)
{
	int16_t rssi = -157 + raw_rssi;

	if (snr_quarter_db < 0) {
		rssi += snr_quarter_db / 4;
	}
	return rssi;
}

bool radio_poll_rx(struct radio_frame *frame)
{
	uint8_t flags = 0, nb = 0, addr = 0, raw_snr = 0, raw_rssi = 0;
	bool got = false;

	k_mutex_lock(&radio_lock, K_FOREVER);
	if (reg_read(REG_IRQ_FLAGS, &flags) == 0 && (flags & IRQ_RX_DONE)) {
		account_dio0();
		if (flags & IRQ_PAYLOAD_CRC_ERR) {
			stats.crc_errors++;
		} else {
			reg_read(REG_RX_NB_BYTES, &nb);
			reg_read(REG_FIFO_RX_CURRENT_ADDR, &addr);
			reg_read(REG_PKT_SNR_VALUE, &raw_snr);
			reg_read(REG_PKT_RSSI_VALUE, &raw_rssi);
			reg_write(REG_FIFO_ADDR_PTR, addr);
			fifo_read(frame->data, nb);
			frame->len = nb;
			frame->snr = (int8_t)raw_snr / 4;
			frame->rssi = packet_rssi(raw_rssi, (int8_t)raw_snr);
			stats.rx_frames++;
			got = true;
		}
		reg_write(REG_IRQ_FLAGS, 0xFF);
	}
	k_mutex_unlock(&radio_lock);
	return got;
}

bool radio_channel_busy(void)
{
	uint8_t stat = 0;

	k_mutex_lock(&radio_lock, K_FOREVER);
	reg_read(REG_MODEM_STAT, &stat);
	k_mutex_unlock(&radio_lock);

	/* bit 0 signal detected, 1 synchronized, 2 RX on-going, 3 header valid */
	return (stat & 0x0B) != 0;
}

int radio_transmit(const uint8_t *data, size_t len)
{
	uint32_t airtime = radio_airtime_ms(len);
	int64_t deadline = k_uptime_get() + airtime + 1000;
	uint8_t flags = 0;
	int ret = 0;

	if (len == 0 || len > 255) {
		return -EINVAL;
	}

	k_mutex_lock(&radio_lock, K_FOREVER);
	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
	ret |= reg_write(REG_DIO_MAPPING_1, DIO0_TX_DONE);
	ret |= reg_write(REG_FIFO_TX_BASE_ADDR, 0x00);
	ret |= reg_write(REG_FIFO_ADDR_PTR, 0x00);
	ret |= fifo_write(data, len);
	ret |= reg_write(REG_PAYLOAD_LENGTH, (uint8_t)len);
	ret |= reg_write(REG_IRQ_FLAGS, 0xFF);
	atomic_set(&dio0_edge, 0);
	k_sem_reset(&irq_sem);
	ret |= reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

	while (ret == 0) {
		k_sem_take(&irq_sem, K_MSEC(5));
		reg_read(REG_IRQ_FLAGS, &flags);
		if (flags & IRQ_TX_DONE) {
			account_dio0();
			stats.tx_frames++;
			stats.tx_airtime_ms += airtime;
			break;
		}
		if (k_uptime_get() > deadline) {
			ret = -ETIMEDOUT;
		}
	}

	start_rx();
	k_mutex_unlock(&radio_lock);

	if (ret != 0) {
		LOG_ERR("TX failed (%d)", ret);
	}
	return ret;
}

uint32_t radio_airtime_ms(size_t len)
{
	/* Semtech AN1200.13 / SX1276 datasheet 4.1.1.7: explicit header, CRC on. */
	uint32_t sf = state.preset->sf;
	uint32_t cr = state.preset->cr - 4;
	uint32_t de = symbol_us() > 16000U ? 1 : 0;
	int32_t num = 8 * (int32_t)len - 4 * (int32_t)sf + 28 + 16;
	int32_t den = 4 * (int32_t)(sf - 2 * de);
	int32_t extra = num > 0 ? ((num + den - 1) / den) * (int32_t)(cr + 4) : 0;
	uint32_t payload_symbols = 8 + (uint32_t)extra;
	uint64_t preamble_us = (uint64_t)(MESHTASTIC_PREAMBLE * 4 + 17) * symbol_us() / 4;

	return (uint32_t)((preamble_us + (uint64_t)payload_symbols * symbol_us()) / 1000U);
}

uint32_t radio_slot_time_ms(void)
{
	/* RadioInterface::computeSlotTimeMsec: 2.5 symbols + propagation and turnaround. */
	return (5U * symbol_us() / 2U) / 1000U + 8U;
}

const struct radio_state *radio_get(void)
{
	return &state;
}

void radio_get_stats(struct radio_stats *s)
{
	k_mutex_lock(&radio_lock, K_FOREVER);
	*s = stats;
	k_mutex_unlock(&radio_lock);
}

/* ---------------------------------------------------------- diagnostics */

int radio_read_diag(struct radio_diag *d)
{
	uint8_t f[3], pre[2], rssi = 0;
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
	ret |= reg_read(REG_PA_CONFIG, &d->pa_config);
	ret |= reg_read(REG_MODEM_STAT, &d->modem_stat);
	ret |= reg_read(REG_RSSI_VALUE, &rssi);
	k_mutex_unlock(&radio_lock);

	d->frf = ((uint32_t)f[0] << 16) | ((uint32_t)f[1] << 8) | f[2];
	d->preamble = ((uint16_t)pre[0] << 8) | pre[1];
	d->rssi_now = -157 + rssi; /* HF port, datasheet 5.5.5 */
	return ret ? -EIO : 0;
}

void radio_expected_diag(struct radio_diag *d)
{
	const struct mesh_preset *p = state.preset;

	memset(d, 0, sizeof(*d));
	d->op_mode = MODE_LONG_RANGE | MODE_RX_CONT;
	d->frf = (uint32_t)(((uint64_t)state.freq_hz << 19) / 32000000U);
	d->modem_config_1 = (bw_code(p->bw_khz) << 4) | ((p->cr - 4) << 1);
	d->modem_config_2 = (p->sf << 4) | 0x04;
	d->modem_config_3 = 0x04 | (symbol_us() > 16000U ? 0x08 : 0x00);
	d->sync_word = MESHTASTIC_SYNC_WORD;
	d->preamble = MESHTASTIC_PREAMBLE;
	d->pa_config = pa_config_value();
}
