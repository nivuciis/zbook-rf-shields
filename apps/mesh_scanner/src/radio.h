/**
 * @file radio.h
 * @brief RFM95W (SX1276) Meshtastic receiver: regions, presets, frequency slot.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RADIO_H_
#define RADIO_H_

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
	uint8_t sync_word;      /* 0x2B = Meshtastic */
};

/** Raw received frame handed to the application. */
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

/** Check the radio and start receiving with the given settings. */
int radio_init(const struct mesh_region *region, const struct mesh_preset *preset);

/** Retune: stop RX, reconfigure, restart RX. */
int radio_set(const struct mesh_region *region, const struct mesh_preset *preset);

const struct radio_state *radio_get(void);

/** Meshtastic sync word. Any other value puts the app in raw test mode. */
#define MESHTASTIC_SYNC_WORD 0x2B

/** Change the LoRa sync word (re-applies the current settings). */
int radio_set_sync_word(uint8_t sync_word);

/** Block until a frame arrives (or timeout). Returns 0 on success. */
int radio_get_frame(struct radio_frame *frame, int32_t timeout_ms);

/** Number of frames received since boot. */
uint32_t radio_rx_count(void);

/** Number of frames dropped because of a payload CRC error. */
uint32_t radio_crc_errors(void);

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
	uint8_t modem_stat;
	uint16_t valid_headers;  /* headers decoded since RX started */
	uint16_t valid_packets;  /* packets with a good CRC since RX started */
	int16_t rssi_now;        /* current channel RSSI, dBm */
};

int radio_read_diag(struct radio_diag *d);

/** Expected register values for the current settings. */
void radio_expected_diag(struct radio_diag *d);

#endif /* RADIO_H_ */
