/**
 * @file meshtastic.c
 * @brief Meshtastic over-the-air packet decoding (receive side only).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "aes128.h"
#include "meshtastic.h"
#include "pb.h"

/* src/mesh/Channels.h, meshtastic/firmware */
const uint8_t mesh_default_key[16] = {
	0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
	0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
};

/* Flags byte layout (src/mesh/RadioInterface.h) */
#define FLAG_HOP_LIMIT_MASK  0x07
#define FLAG_WANT_ACK_MASK   0x08
#define FLAG_VIA_MQTT_MASK   0x10
#define FLAG_HOP_START_SHIFT 5

uint32_t mesh_djb2(const char *str)
{
	uint32_t h = 5381;

	while (*str) {
		h = (h << 5) + h + (uint8_t)*str++;
	}
	return h;
}

uint8_t mesh_channel_hash(const char *channel_name, const uint8_t *key, size_t key_len)
{
	uint8_t h = 0;

	while (*channel_name) {
		h ^= (uint8_t)*channel_name++;
	}
	for (size_t i = 0; i < key_len; i++) {
		h ^= key[i];
	}
	return h;
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

static void decode_user(const uint8_t *buf, size_t len, struct mesh_packet *out)
{
	struct pb_reader r;
	struct pb_field f;

	memset(&out->info.user, 0, sizeof(out->info.user));
	pb_reader_init(&r, buf, len);
	while (pb_next(&r, &f)) {
		switch (f.num) {
		case 1: pb_copy_string(&f, out->info.user.id, sizeof(out->info.user.id)); break;
		case 2: pb_copy_string(&f, out->info.user.long_name, sizeof(out->info.user.long_name)); break;
		case 3: pb_copy_string(&f, out->info.user.short_name, sizeof(out->info.user.short_name)); break;
		case 5: out->info.user.hw_model = (uint32_t)f.value; break;
		case 7: out->info.user.role = (uint32_t)f.value; break;
		default: break;
		}
	}
	out->kind = MESH_INFO_USER;
}

static void decode_position(const uint8_t *buf, size_t len, struct mesh_packet *out)
{
	struct pb_reader r;
	struct pb_field f;
	bool has_lat = false, has_lon = false;

	memset(&out->info.pos, 0, sizeof(out->info.pos));
	pb_reader_init(&r, buf, len);
	while (pb_next(&r, &f)) {
		switch (f.num) {
		case 1: out->info.pos.lat_i = (int32_t)(uint32_t)f.value; has_lat = true; break;
		case 2: out->info.pos.lon_i = (int32_t)(uint32_t)f.value; has_lon = true; break;
		case 3: out->info.pos.alt_m = (int32_t)f.value; out->info.pos.has_alt = true; break;
		default: break;
		}
	}
	/* Nodes that share no location send an empty Position. */
	if (has_lat && has_lon && (out->info.pos.lat_i != 0 || out->info.pos.lon_i != 0)) {
		out->kind = MESH_INFO_POSITION;
	}
}

static void decode_telemetry(const uint8_t *buf, size_t len, struct mesh_packet *out)
{
	struct pb_reader r, dm;
	struct pb_field f, g;

	pb_reader_init(&r, buf, len);
	while (pb_next(&r, &f)) {
		if (f.num != 2 || f.wire != PB_LEN) { /* device_metrics */
			continue;
		}
		pb_reader_init(&dm, f.data, f.len);
		while (pb_next(&dm, &g)) {
			if (g.num == 1 && g.wire == PB_VARINT) { /* battery_level */
				out->info.battery_level = (uint32_t)g.value;
				out->kind = MESH_INFO_BATTERY;
			}
		}
	}
}

static bool decode_data(const uint8_t *buf, size_t len, struct mesh_packet *out)
{
	struct pb_reader r;
	struct pb_field f;
	const uint8_t *payload = NULL;
	size_t payload_len = 0;
	bool has_port = false;

	pb_reader_init(&r, buf, len);
	while (pb_next(&r, &f)) {
		if (f.num == 1 && f.wire == PB_VARINT) {
			out->portnum = (uint32_t)f.value;
			has_port = true;
		} else if (f.num == 2 && f.wire == PB_LEN) {
			payload = f.data;
			payload_len = f.len;
		}
	}
	/* A wrong key yields random bytes that rarely parse cleanly. */
	if (r.error || !has_port || out->portnum == 0 || out->portnum > 511) {
		return false;
	}

	switch (out->portnum) {
	case PORT_NODEINFO:
		decode_user(payload, payload_len, out);
		break;
	case PORT_POSITION:
		decode_position(payload, payload_len, out);
		break;
	case PORT_TELEMETRY:
		decode_telemetry(payload, payload_len, out);
		break;
	case PORT_TEXT_MESSAGE: {
		size_t n = payload_len < sizeof(out->info.text) - 1 ? payload_len
								      : sizeof(out->info.text) - 1;
		if (payload) {
			memcpy(out->info.text, payload, n);
		}
		out->info.text[n] = '\0';
		out->kind = MESH_INFO_TEXT;
		break;
	}
	default:
		break;
	}
	return true;
}

int mesh_decode(uint8_t *buf, size_t len, uint8_t channel_hash, const uint8_t key[16],
		struct mesh_packet *out)
{
	memset(out, 0, sizeof(*out));
	if (len < MESH_HEADER_LEN) {
		return -1;
	}

	out->to = le32(&buf[0]);
	out->from = le32(&buf[4]);
	out->id = le32(&buf[8]);
	out->hop_limit = buf[12] & FLAG_HOP_LIMIT_MASK;
	out->want_ack = (buf[12] & FLAG_WANT_ACK_MASK) != 0;
	out->via_mqtt = (buf[12] & FLAG_VIA_MQTT_MASK) != 0;
	out->hop_start = buf[12] >> FLAG_HOP_START_SHIFT;
	out->channel_hash = buf[13];
	out->next_hop = buf[14];
	out->relay_node = buf[15];

	if (out->channel_hash != channel_hash || len == MESH_HEADER_LEN) {
		return 0; /* not our channel (or a PKI direct message): header only */
	}

	/* Nonce (CryptoEngine::initNonce): packet id as uint64 LE, then sender as uint32 LE. */
	uint8_t nonce[16] = {0};
	struct aes128_ctx aes;

	memcpy(&nonce[0], &buf[8], 4);
	memcpy(&nonce[8], &buf[4], 4);
	aes128_init(&aes, key);
	aes128_ctr_xcrypt(&aes, nonce, &buf[MESH_HEADER_LEN], len - MESH_HEADER_LEN);

	out->decrypted = decode_data(&buf[MESH_HEADER_LEN], len - MESH_HEADER_LEN, out);
	return 0;
}

const char *mesh_port_name(uint32_t portnum)
{
	switch (portnum) {
	case PORT_TEXT_MESSAGE: return "TEXT";
	case PORT_POSITION: return "POSITION";
	case PORT_NODEINFO: return "NODEINFO";
	case PORT_ROUTING: return "ROUTING";
	case PORT_TELEMETRY: return "TELEMETRY";
	case PORT_TRACEROUTE: return "TRACEROUTE";
	case PORT_NEIGHBORINFO: return "NEIGHBORINFO";
	case PORT_MAP_REPORT: return "MAP_REPORT";
	default: return "OTHER";
	}
}

const char *mesh_role_name(uint32_t role)
{
	/* meshtastic/config.proto, Config.DeviceConfig.Role */
	static const char *const names[] = {
		"CLIENT", "CLIENT_MUTE", "ROUTER", "ROUTER_CLIENT", "REPEATER",
		"TRACKER", "SENSOR", "TAK", "CLIENT_HIDDEN", "LOST_AND_FOUND",
		"TAK_TRACKER", "ROUTER_LATE", "CLIENT_BASE",
	};

	return role < sizeof(names) / sizeof(names[0]) ? names[role] : "?";
}
