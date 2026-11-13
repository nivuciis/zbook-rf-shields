/**
 * @file mesh.h
 * @brief Meshtastic node: identity, flooding router, sending and receiving.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MESH_H_
#define MESH_H_

#include <stdbool.h>
#include <stdint.h>

struct mesh_identity {
	uint32_t num;            /* node number, from the chip unique ID */
	char long_name[40];
	char short_name[5];
};

struct mesh_stats {
	uint32_t received;       /* new packets (duplicates excluded) */
	uint32_t decrypted;
	uint32_t other_channel;
	uint32_t duplicates;
	uint32_t relayed;
	uint32_t relays_cancelled;
	uint32_t sent;           /* our own packets */
};

/** Initialize the identity and start the mesh thread (radio must be up). */
void mesh_start(void);

const struct mesh_identity *mesh_identity(void);
void mesh_set_name(const char *long_name, const char *short_name);

/** Queue packets for transmission. Return 0 or a negative errno. */
int mesh_send_text(uint32_t to, const char *text, bool want_ack, uint32_t *id_out);
int mesh_send_nodeinfo(uint32_t to, bool want_response);
int mesh_send_position(uint32_t to);

/** Fixed position to announce (degrees * 1e7). */
void mesh_set_position(int32_t lat_i, int32_t lon_i, int32_t alt_m);
void mesh_clear_position(void);
bool mesh_has_position(void);

void mesh_set_relay(bool on);
bool mesh_relay_enabled(void);
void mesh_set_live(bool on);

void mesh_get_stats(struct mesh_stats *s);

#endif /* MESH_H_ */
