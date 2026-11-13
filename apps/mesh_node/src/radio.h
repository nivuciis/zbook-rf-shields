/**
 * @file radio.h
 * @brief RFM95W (SX1276) LoRa transceiver for Meshtastic, driven over SPI.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Not thread safe by design for RX/TX: radio_poll_rx(), radio_transmit() and
 * radio_wait_event() are meant to be called from a single thread (the mesh
 * thread). Configuration and diagnostics may be called from any thread.
 */
#ifndef RADIO_H_
#define RADIO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Meshtastic region (src/mesh/RadioInterface.cpp, RDEF table). */
struct mesh_region {
	const char *name;
	uint32_t start_hz;
	uint32_t end_hz;
};

/** Meshtastic modem preset (src/mesh/MeshRadio.h, modemPresetToParams). */
struct mesh_preset {
	const char *name;    /* display name, also the default channel name */
	uint16_t bw_khz;
	uint8_t sf;
	uint8_t cr;          /* 5..8 -> 4/5..4/8 */
};

struct radio_state {
	const struct mesh_region *region;
	const struct mesh_preset *preset;
	uint32_t freq_hz;
	uint32_t num_slots;
	uint32_t slot;          /* 0-based */
	uint8_t channel_hash;
	uint8_t tx_power_dbm;
};

/** A received frame. */
struct radio_frame {
	uint16_t len;
	int16_t rssi;
	int8_t snr;
	uint8_t data[256];
};

extern const struct mesh_region mesh_regions[];
extern const size_t mesh_region_count;
extern const struct mesh_preset mesh_presets[];
extern const size_t mesh_preset_count;

const struct mesh_region *radio_find_region(const char *name);
const struct mesh_preset *radio_find_preset(const char *name);

/** Reset and check the chip, set up DIO0 (if wired) and start receiving. */
int radio_init(const struct mesh_region *region, const struct mesh_preset *preset);

/** Retune and restart RX. */
int radio_set(const struct mesh_region *region, const struct mesh_preset *preset);

/** TX power on PA_BOOST, 2..17 dBm. */
int radio_set_tx_power(uint8_t dbm);

const struct radio_state *radio_get(void);

/** Sleep until DIO0 fires or the timeout expires (polling fallback). */
void radio_wait_event(int32_t timeout_ms);

/** If a packet is waiting in the FIFO, copy it out. Returns true if one was read. */
bool radio_poll_rx(struct radio_frame *frame);

/** True while the modem is detecting or receiving a LoRa transmission. */
bool radio_channel_busy(void);

/** Transmit a frame (blocks until TxDone), then go back to RX. */
int radio_transmit(const uint8_t *data, size_t len);

/** Time on air of a frame of len bytes with the current settings. */
uint32_t radio_airtime_ms(size_t len);

/** Meshtastic slot time for the current preset (contention window unit). */
uint32_t radio_slot_time_ms(void);

struct radio_stats {
	uint32_t rx_frames;
	uint32_t crc_errors;
	uint32_t tx_frames;
	uint32_t tx_airtime_ms;
	uint32_t dio0_hits;    /* events signalled by the DIO0 interrupt */
	uint32_t dio0_misses;  /* events found by polling with no DIO0 edge */
	bool dio0_configured;
};

void radio_get_stats(struct radio_stats *s);

/** Raw SX1276 registers, read back from the chip for diagnostics. */
struct radio_diag {
	uint8_t version;
	uint8_t op_mode;
	uint32_t frf;
	uint8_t modem_config_1;
	uint8_t modem_config_2;
	uint8_t modem_config_3;
	uint8_t sync_word;
	uint16_t preamble;
	uint8_t pa_config;
	uint8_t modem_stat;
	int16_t rssi_now;
};

int radio_read_diag(struct radio_diag *d);

/** Expected register values for the current settings. */
void radio_expected_diag(struct radio_diag *d);

#endif /* RADIO_H_ */
