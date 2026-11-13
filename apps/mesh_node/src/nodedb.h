/**
 * @file nodedb.h
 * @brief Table of Meshtastic nodes heard over the air.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NODEDB_H_
#define NODEDB_H_

#include <stdbool.h>
#include <stdint.h>

#include "meshtastic.h"

#define NODEDB_MAX_NODES 64

struct node_entry {
	uint32_t num;
	char long_name[40];
	char short_name[8];
	uint32_t hw_model;
	uint32_t role;
	bool has_user;

	int64_t last_heard_ms;   /* k_uptime_get() */
	uint32_t packets;
	uint32_t last_id;

	int16_t rssi;            /* last direct (0-hop) reception */
	int8_t snr;
	bool has_signal;
	int8_t hops_away;        /* -1 unknown */
	uint8_t last_relay;      /* relay_node byte of the last relayed packet */

	int32_t lat_i;
	int32_t lon_i;
	int32_t alt_m;
	bool has_pos;

	uint32_t battery_level;
	bool has_battery;

	uint32_t last_port;
	bool last_decrypted;
};

/** Update the table with a decoded packet. Returns true for a new node. */
bool nodedb_update(const struct mesh_packet *pkt, int16_t rssi, int8_t snr);

/**
 * Copy the table, most recently heard first.
 * @return number of entries copied.
 */
int nodedb_snapshot(struct node_entry *out, int max);

/** Copy one node. Returns false if unknown. */
bool nodedb_get(uint32_t num, struct node_entry *out);

void nodedb_clear(void);

#endif /* NODEDB_H_ */
