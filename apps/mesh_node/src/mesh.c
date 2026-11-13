/**
 * @file mesh.c
 * @brief Meshtastic node: identity, flooding router, sending and receiving.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Behaves like a Meshtastic CLIENT node on the primary channel (default key):
 *  - managed flooding (src/mesh/FloodingRouter.cpp): every new packet that
 *    still has hops left is rebroadcast after an SNR-weighted random delay,
 *    and the rebroadcast is cancelled if another node relays it first;
 *  - duplicate detection by (sender, packet id);
 *  - routing ACKs for packets addressed to us that ask for one;
 *  - NODEINFO replies when a node asks for them (want_response).
 * Public-key (PKI) direct messages are relayed but cannot be read.
 */
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "mesh.h"
#include "meshtastic.h"
#include "nodedb.h"
#include "radio.h"

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

#define HOP_LIMIT          3
#define CW_MIN             3     /* RadioInterface.h */
#define CW_MAX             8
#define TX_QUEUE_LEN       8
#define HISTORY_LEN        64
#define PENDING_ACKS       4
#define NODEINFO_REPLY_GAP_MS (5 * 60 * 1000)
#define HW_MODEL_PRIVATE   255   /* HardwareModel.PRIVATE_HW */
#define ROLE_CLIENT        0

struct tx_item {
	bool active;
	bool relay;
	uint32_t from;
	uint32_t id;
	int64_t due_ms;
	uint16_t len;
	uint8_t data[256];
};

struct seen {
	uint32_t from;
	uint32_t id;
};

static struct mesh_identity me;
static struct mesh_stats stats;
static bool relay_on = true;
static bool live_on = true;

static int32_t pos_lat, pos_lon, pos_alt;
static bool pos_set;

static struct tx_item txq[TX_QUEUE_LEN];
static struct seen history[HISTORY_LEN];
static size_t history_next;
static uint32_t pending_acks[PENDING_ACKS];
static size_t pending_next;
static struct {
	uint32_t node;
	int64_t at_ms;
} nodeinfo_replies[8];

static K_MUTEX_DEFINE(lock);
static K_SEM_DEFINE(started, 0, 1);

/* ------------------------------------------------------------- helpers */

static uint32_t rng_state;

static uint32_t rand32(void)
{
	/* xorshift32, seeded from the chip ID and the cycle counter */
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return rng_state;
}

static uint32_t rand_range(uint32_t n)
{
	return n ? rand32() % n : 0;
}

static uint32_t next_packet_id(void)
{
	static uint32_t id;

	if (id == 0) {
		id = rand32() & 0x7fffffff;
	}
	return ++id;
}

static uint8_t our_byte(void)
{
	return (uint8_t)(me.num & 0xFF);
}

static bool history_seen(uint32_t from, uint32_t id)
{
	for (size_t i = 0; i < HISTORY_LEN; i++) {
		if (history[i].id == id && history[i].from == from) {
			return true;
		}
	}
	return false;
}

static void history_add(uint32_t from, uint32_t id)
{
	history[history_next].from = from;
	history[history_next].id = id;
	history_next = (history_next + 1) % HISTORY_LEN;
}

/* SNR-weighted rebroadcast delay (RadioInterface::getTxDelayMsecWeighted, CLIENT):
 * nodes that heard the packet weakly (likely farther away) relay first.
 */
static uint32_t relay_delay_ms(int8_t snr)
{
	int32_t cw = CW_MIN + ((int32_t)snr + 20) * (CW_MAX - CW_MIN) / 30;

	cw = CLAMP(cw, CW_MIN, CW_MAX);
	return (2U * CW_MAX + rand_range(1U << cw)) * radio_slot_time_ms();
}

/* Random delay before sending our own packets (RadioInterface::getTxDelayMsec). */
static uint32_t send_delay_ms(void)
{
	return rand_range(1U << CW_MIN) * radio_slot_time_ms();
}

/* ------------------------------------------------------------ TX queue */

static int enqueue(const uint8_t *frame, size_t len, bool relay, uint32_t from, uint32_t id,
		   uint32_t delay_ms)
{
	int ret = -ENOMEM;

	k_mutex_lock(&lock, K_FOREVER);
	for (size_t i = 0; i < TX_QUEUE_LEN; i++) {
		if (!txq[i].active) {
			txq[i].active = true;
			txq[i].relay = relay;
			txq[i].from = from;
			txq[i].id = id;
			txq[i].due_ms = k_uptime_get() + delay_ms;
			txq[i].len = (uint16_t)len;
			memcpy(txq[i].data, frame, len);
			ret = 0;
			break;
		}
	}
	k_mutex_unlock(&lock);

	if (ret != 0) {
		LOG_WRN("TX queue full, packet dropped");
	}
	return ret;
}

static void cancel_relay(uint32_t from, uint32_t id)
{
	k_mutex_lock(&lock, K_FOREVER);
	for (size_t i = 0; i < TX_QUEUE_LEN; i++) {
		if (txq[i].active && txq[i].relay && txq[i].from == from && txq[i].id == id) {
			txq[i].active = false;
			stats.relays_cancelled++;
		}
	}
	k_mutex_unlock(&lock);
}

/* Build and queue one of our own packets on the primary channel. */
static int send_data(uint32_t to, uint32_t portnum, const uint8_t *payload, size_t payload_len,
		     bool want_ack, bool want_response, uint32_t request_id, uint32_t *id_out)
{
	uint8_t frame[256];
	struct mesh_tx_header hdr = {
		.to = to,
		.from = me.num,
		.id = next_packet_id(),
		.hop_limit = HOP_LIMIT,
		.want_ack = want_ack,
		.channel_hash = radio_get()->channel_hash,
		.relay_node = our_byte(),
	};
	struct mesh_tx_data data = {
		.portnum = portnum,
		.payload = payload,
		.payload_len = payload_len,
		.want_response = want_response,
		.request_id = request_id,
	};
	int len = mesh_encode(frame, sizeof(frame), &hdr, &data, mesh_default_key);

	if (len < 0) {
		return -EMSGSIZE;
	}
	if (id_out) {
		*id_out = hdr.id;
	}
	return enqueue(frame, (size_t)len, false, me.num, hdr.id, send_delay_ms());
}

static void service_tx(void)
{
	static struct tx_item item;
	int64_t now = k_uptime_get();
	int best = -1;

	k_mutex_lock(&lock, K_FOREVER);
	for (size_t i = 0; i < TX_QUEUE_LEN; i++) {
		if (txq[i].active && txq[i].due_ms <= now &&
		    (best < 0 || txq[i].due_ms < txq[best].due_ms)) {
			best = (int)i;
		}
	}
	if (best >= 0) {
		item = txq[best];
		txq[best].active = false;
	}
	k_mutex_unlock(&lock);

	if (best < 0) {
		return;
	}

		/* Listen before talk: someone is transmitting, try again a bit later.
	 * After several attempts in a row, send anyway rather than starve.
	 */
	static int busy_streak;

	if (radio_channel_busy() && busy_streak < 10) {
		busy_streak++;
		LOG_DBG("Channel busy, retrying later (%d)", busy_streak);
		enqueue(item.data, item.len, item.relay, item.from, item.id,
			radio_airtime_ms(64) + send_delay_ms());
		return;
	}
	
	busy_streak = 0;

	if (radio_transmit(item.data, item.len) == 0) {
		if (item.relay) {
			stats.relayed++;
			LOG_DBG("Relayed !%08x id %08x", item.from, item.id);
		} else {
			stats.sent++;
			history_add(item.from, item.id); /* so our own echoes are not new */
		}
	}
}

/* --------------------------------------------------------------- receive */

static const char *name_of(uint32_t num, char *buf, size_t size)
{
	struct node_entry n;

	if (nodedb_get(num, &n) && n.has_user) {
		snprintf(buf, size, "%s", n.short_name);
	} else {
		snprintf(buf, size, "!%08x", num);
	}
	return buf;
}

static void print_packet(const struct mesh_packet *p, const struct radio_frame *f)
{
	char who[12];
	int hops = p->hop_start >= p->hop_limit && p->hop_start ? p->hop_start - p->hop_limit : -1;

	name_of(p->from, who, sizeof(who));
	if (!p->decrypted) {
		LOG_INF("%-9s ch=0x%02x (other channel/PKI) hops=%d %d dBm %d dB", who,
			p->channel_hash, hops, f->rssi, f->snr);
		return;
	}
	switch (p->kind) {
	case MESH_INFO_USER:
		LOG_INF("%-9s NODEINFO \"%s\" %s hops=%d %d dBm %d dB", who,
			p->info.user.long_name, mesh_hw_model_name(p->info.user.hw_model), hops,
			f->rssi, f->snr);
		break;
	case MESH_INFO_TEXT:
		LOG_INF("%-9s %s: \"%s\" hops=%d %d dBm %d dB", who,
			p->to == me.num ? "DM to you" : "TEXT", p->info.text, hops, f->rssi, f->snr);
		break;
	case MESH_INFO_BATTERY:
		LOG_INF("%-9s TELEMETRY battery=%u%% hops=%d %d dBm %d dB", who,
			p->info.battery_level, hops, f->rssi, f->snr);
		break;
	default:
		LOG_INF("%-9s %s hops=%d %d dBm %d dB", who, mesh_port_name(p->portnum), hops,
			f->rssi, f->snr);
		break;
	}
}

static bool nodeinfo_reply_allowed(uint32_t node)
{
	int64_t now = k_uptime_get();
	size_t oldest = 0;

	for (size_t i = 0; i < ARRAY_SIZE(nodeinfo_replies); i++) {
		if (nodeinfo_replies[i].node == node) {
			if (now - nodeinfo_replies[i].at_ms < NODEINFO_REPLY_GAP_MS) {
				return false;
			}
			nodeinfo_replies[i].at_ms = now;
			return true;
		}
		if (nodeinfo_replies[i].at_ms < nodeinfo_replies[oldest].at_ms) {
			oldest = i;
		}
	}
	nodeinfo_replies[oldest].node = node;
	nodeinfo_replies[oldest].at_ms = now;
	return true;
}

static void check_ack(const struct mesh_packet *p)
{
	char who[12];

	k_mutex_lock(&lock, K_FOREVER);
	for (size_t i = 0; i < PENDING_ACKS; i++) {
		if (pending_acks[i] != 0 && pending_acks[i] == p->request_id) {
			pending_acks[i] = 0;
			k_mutex_unlock(&lock);
			if (p->info.routing_error == 0) {
				LOG_INF("ACK from %s for message id %08x", name_of(p->from, who, sizeof(who)),
					p->request_id);
			} else {
				LOG_WRN("NAK from %s for id %08x (error %u)",
					name_of(p->from, who, sizeof(who)), p->request_id,
					p->info.routing_error);
			}
			return;
		}
	}
	k_mutex_unlock(&lock);
}

static void deliver(const struct mesh_packet *p)
{
	bool for_us = p->to == me.num;
	bool broadcast = p->to == MESH_BROADCAST;

	if (!p->decrypted || !(for_us || broadcast)) {
		return;
	}

	if (p->kind == MESH_INFO_ROUTING && for_us) {
		check_ack(p);
	}

	if (p->portnum == PORT_NODEINFO && p->want_response && nodeinfo_reply_allowed(p->from)) {
		mesh_send_nodeinfo(p->from, false);
	}

	if (for_us && p->want_ack && p->portnum != PORT_ROUTING) {
		uint8_t ack[8];
		size_t n = mesh_build_routing_ack(ack, sizeof(ack));

		send_data(p->from, PORT_ROUTING, ack, n, false, false, p->id, NULL);
	}
}

static void maybe_relay(const struct radio_frame *raw, const struct mesh_packet *p)
{
	uint8_t frame[256];

	if (!relay_on || p->to == me.num || p->hop_limit == 0) {
		return;
	}
	/* Next-hop routing (2.6+): only the chosen relay forwards directed packets. */
	if (p->next_hop != 0 && p->next_hop != our_byte()) {
		return;
	}

	/* Rebroadcast the original encrypted bytes, one hop less, with us as relay. */
	memcpy(frame, raw->data, raw->len);
	frame[12] = (frame[12] & ~0x07) | ((p->hop_limit - 1) & 0x07);
	frame[15] = our_byte();
	enqueue(frame, raw->len, true, p->from, p->id, relay_delay_ms(raw->snr));
}

static void handle_rx(const struct radio_frame *f)
{
	static struct mesh_packet pkt;
	static uint8_t buf[256];
	char who[12];

	memcpy(buf, f->data, f->len);
	if (mesh_decode(buf, f->len, radio_get()->channel_hash, mesh_default_key, &pkt) < 0) {
		return; /* shorter than a Meshtastic header */
	}

	if (pkt.from == me.num) {
		/* Our own packet coming back: another node relayed it (implicit ACK). */
		if (live_on && pkt.relay_node != our_byte()) {
			LOG_INF("Our packet %08x was relayed by node ..%02x", pkt.id, pkt.relay_node);
		}
		return;
	}

	if (history_seen(pkt.from, pkt.id)) {
		stats.duplicates++;
		cancel_relay(pkt.from, pkt.id); /* someone else already relayed it */
		nodedb_update(&pkt, f->rssi, f->snr);
		return;
	}
	history_add(pkt.from, pkt.id);

	stats.received++;
	if (pkt.decrypted) {
		stats.decrypted++;
	} else {
		stats.other_channel++;
	}

	if (nodedb_update(&pkt, f->rssi, f->snr)) {
		LOG_INF("New node: %s", name_of(pkt.from, who, sizeof(who)));
	}
	if (live_on || (pkt.kind == MESH_INFO_TEXT)) {
		print_packet(&pkt, f);
	}

	deliver(&pkt);
	maybe_relay(f, &pkt);
}

/* --------------------------------------------------------------- thread */

static void mesh_thread(void *a, void *b, void *c)
{
	static struct radio_frame frame;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	k_sem_take(&started, K_FOREVER);
	while (1) {
		radio_wait_event(10);
		if (radio_poll_rx(&frame)) {
			handle_rx(&frame);
		}
		service_tx();
	}
}

K_THREAD_DEFINE(mesh_tid, 4096, mesh_thread, NULL, NULL, NULL, 6, 0, 0);

/* ------------------------------------------------------------ public API */

void mesh_start(void)
{
	uint8_t id[16] = {0};
	ssize_t n = hwinfo_get_device_id(id, sizeof(id));

	/* Node number: last 4 bytes of the chip unique ID (like the MAC on an ESP32). */
	if (n >= 4) {
		me.num = sys_get_be32(&id[n - 4]);
	}
	if (me.num == 0 || me.num == MESH_BROADCAST) {
		me.num = 0x2b2b0000 | (k_cycle_get_32() & 0xFFFF);
	}
	rng_state = me.num ^ k_cycle_get_32() ^ 0x9e3779b9;
	if (rng_state == 0) {
		rng_state = 0x1234567;
	}

	snprintf(me.short_name, sizeof(me.short_name), "%04x", me.num & 0xFFFF);
	snprintf(me.long_name, sizeof(me.long_name), "ZBook %s", me.short_name);

	LOG_INF("This node: !%08x \"%s\" (%s)", me.num, me.long_name, me.short_name);
	k_sem_give(&started);
}

const struct mesh_identity *mesh_identity(void)
{
	return &me;
}

void mesh_set_name(const char *long_name, const char *short_name)
{
	k_mutex_lock(&lock, K_FOREVER);
	snprintf(me.long_name, sizeof(me.long_name), "%s", long_name);
	if (short_name) {
		snprintf(me.short_name, sizeof(me.short_name), "%s", short_name);
	}
	k_mutex_unlock(&lock);
}

int mesh_send_text(uint32_t to, const char *text, bool want_ack, uint32_t *id_out)
{
	uint32_t id = 0;
	size_t len = strlen(text);
	int ret;

	if (len == 0 || len > 200) {
		return -EINVAL;
	}
	ret = send_data(to, PORT_TEXT_MESSAGE, (const uint8_t *)text, len, want_ack, false, 0, &id);
	if (ret == 0 && want_ack) {
		k_mutex_lock(&lock, K_FOREVER);
		pending_acks[pending_next] = id;
		pending_next = (pending_next + 1) % PENDING_ACKS;
		k_mutex_unlock(&lock);
	}
	if (id_out) {
		*id_out = id;
	}
	return ret;
}

int mesh_send_nodeinfo(uint32_t to, bool want_response)
{
	uint8_t payload[96];
	size_t n;

	k_mutex_lock(&lock, K_FOREVER);
	n = mesh_build_user(payload, sizeof(payload), me.num, me.long_name, me.short_name,
			    HW_MODEL_PRIVATE, ROLE_CLIENT);
	k_mutex_unlock(&lock);
	if (n == 0) {
		return -EMSGSIZE;
	}
	return send_data(to, PORT_NODEINFO, payload, n, false, want_response, 0, NULL);
}

int mesh_send_position(uint32_t to)
{
	uint8_t payload[32];
	size_t n;

	if (!pos_set) {
		return -ENODATA;
	}
	n = mesh_build_position(payload, sizeof(payload), pos_lat, pos_lon, pos_alt, true);
	return send_data(to, PORT_POSITION, payload, n, false, false, 0, NULL);
}

void mesh_set_position(int32_t lat_i, int32_t lon_i, int32_t alt_m)
{
	pos_lat = lat_i;
	pos_lon = lon_i;
	pos_alt = alt_m;
	pos_set = true;
}

void mesh_clear_position(void)
{
	pos_set = false;
}

bool mesh_has_position(void)
{
	return pos_set;
}

void mesh_set_relay(bool on)
{
	relay_on = on;
}

bool mesh_relay_enabled(void)
{
	return relay_on;
}

void mesh_set_live(bool on)
{
	live_on = on;
}

void mesh_get_stats(struct mesh_stats *s)
{
	*s = stats;
}
