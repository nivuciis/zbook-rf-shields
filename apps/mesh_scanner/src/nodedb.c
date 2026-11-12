/**
 * @file nodedb.c
 * @brief Table of Meshtastic nodes heard over the air.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include <zephyr/kernel.h>

#include "nodedb.h"

static struct node_entry nodes[NODEDB_MAX_NODES];
static int node_count;
static K_MUTEX_DEFINE(lock);

static struct node_entry *find_or_add(uint32_t num, bool *is_new)
{
	int oldest = 0;

	*is_new = false;
	for (int i = 0; i < node_count; i++) {
		if (nodes[i].num == num) {
			return &nodes[i];
		}
		if (nodes[i].last_heard_ms < nodes[oldest].last_heard_ms) {
			oldest = i;
		}
	}

	/* New node: append, or replace the one heard longest ago. */
	int slot = node_count < NODEDB_MAX_NODES ? node_count++ : oldest;

	memset(&nodes[slot], 0, sizeof(nodes[slot]));
	nodes[slot].num = num;
	nodes[slot].hops_away = -1;
	*is_new = true;
	return &nodes[slot];
}

bool nodedb_update(const struct mesh_packet *pkt, int16_t rssi, int8_t snr)
{
	bool is_new;
	int hops = (pkt->hop_start >= pkt->hop_limit && pkt->hop_start != 0)
			   ? pkt->hop_start - pkt->hop_limit
			   : -1;

	k_mutex_lock(&lock, K_FOREVER);

	struct node_entry *n = find_or_add(pkt->from, &is_new);
	bool duplicate = !is_new && n->last_id == pkt->id;

	n->last_heard_ms = k_uptime_get();
	if (!duplicate) {
		n->packets++;
		n->last_id = pkt->id;
		n->last_port = pkt->portnum;
		n->last_decrypted = pkt->decrypted;
	}

	/* Keep the shortest path seen; RSSI/SNR only mean something for 0 hops. */
	if (hops >= 0 && (n->hops_away < 0 || hops < n->hops_away || !duplicate)) {
		n->hops_away = (int8_t)hops;
	}
	if (hops == 0) {
		n->rssi = rssi;
		n->snr = snr;
		n->has_signal = true;
	} else {
		n->last_relay = pkt->relay_node;
	}

	switch (pkt->kind) {
	case MESH_INFO_USER:
		strncpy(n->long_name, pkt->info.user.long_name, sizeof(n->long_name) - 1);
		strncpy(n->short_name, pkt->info.user.short_name, sizeof(n->short_name) - 1);
		n->hw_model = pkt->info.user.hw_model;
		n->role = pkt->info.user.role;
		n->has_user = true;
		break;
	case MESH_INFO_POSITION:
		n->lat_i = pkt->info.pos.lat_i;
		n->lon_i = pkt->info.pos.lon_i;
		n->alt_m = pkt->info.pos.alt_m;
		n->has_pos = true;
		break;
	case MESH_INFO_BATTERY:
		n->battery_level = pkt->info.battery_level;
		n->has_battery = true;
		break;
	default:
		break;
	}

	k_mutex_unlock(&lock);
	return is_new;
}

int nodedb_snapshot(struct node_entry *out, int max)
{
	int n;

	k_mutex_lock(&lock, K_FOREVER);
	n = MIN(node_count, max);
	memcpy(out, nodes, n * sizeof(*out));
	k_mutex_unlock(&lock);

	/* Insertion sort, most recently heard first (n is small). */
	for (int i = 1; i < n; i++) {
		struct node_entry tmp = out[i];
		int j = i - 1;

		while (j >= 0 && out[j].last_heard_ms < tmp.last_heard_ms) {
			out[j + 1] = out[j];
			j--;
		}
		out[j + 1] = tmp;
	}
	return n;
}

bool nodedb_get(uint32_t num, struct node_entry *out)
{
	bool found = false;

	k_mutex_lock(&lock, K_FOREVER);
	for (int i = 0; i < node_count; i++) {
		if (nodes[i].num == num) {
			*out = nodes[i];
			found = true;
			break;
		}
	}
	k_mutex_unlock(&lock);
	return found;
}

void nodedb_clear(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	node_count = 0;
	k_mutex_unlock(&lock);
}
