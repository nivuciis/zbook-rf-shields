/**
 * @file meshtastic.h
 * @brief Meshtastic over-the-air packets: decoding and encoding.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Over-the-air frame (src/mesh/RadioInterface.h, meshtastic/firmware):
 *   16-byte header, all fields little-endian:
 *     to(4) from(4) id(4) flags(1) channel_hash(1) next_hop(1) relay_node(1)
 *   then the payload: a `Data` protobuf, AES-CTR encrypted with the channel key.
 */
#ifndef MESHTASTIC_H_
#define MESHTASTIC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MESH_HEADER_LEN   16
#define MESH_BROADCAST    0xFFFFFFFFu

/* PortNum values (meshtastic/portnums.proto) */
#define PORT_TEXT_MESSAGE 1
#define PORT_POSITION     3
#define PORT_NODEINFO     4
#define PORT_ROUTING      5
#define PORT_TELEMETRY    67
#define PORT_TRACEROUTE   70
#define PORT_NEIGHBORINFO 71
#define PORT_MAP_REPORT   73

enum mesh_info_kind {
	MESH_INFO_NONE = 0,
	MESH_INFO_USER,
	MESH_INFO_POSITION,
	MESH_INFO_TEXT,
	MESH_INFO_BATTERY,
	MESH_INFO_ROUTING,
};

struct mesh_packet {
	/* Header */
	uint32_t to;
	uint32_t from;
	uint32_t id;
	uint8_t hop_limit;
	uint8_t hop_start;
	bool want_ack;
	bool via_mqtt;
	uint8_t channel_hash;
	uint8_t next_hop;
	uint8_t relay_node;

	/* Payload */
	bool decrypted;   /* decrypted with our channel key and parsed as Data */
	uint32_t portnum;
	bool want_response;
	uint32_t request_id;    /* for replies and ACKs: id of the original packet */
	enum mesh_info_kind kind;
	union {
		struct {
			char id[16];
			char long_name[40];
			char short_name[8];
			uint32_t hw_model;
			uint32_t role;
		} user;
		struct {
			int32_t lat_i;   /* degrees * 1e7 */
			int32_t lon_i;   /* degrees * 1e7 */
			int32_t alt_m;
			bool has_alt;
		} pos;
		char text[200];
		uint32_t battery_level; /* percent; 101 = powered */
		uint32_t routing_error; /* Routing.error_reason, 0 = ACK */
	} info;
};

/** djb2 hash used by Meshtastic to pick the frequency slot from the channel name. */
uint32_t mesh_djb2(const char *str);

/** Channel hash: XOR of the channel name bytes XOR the key bytes. */
uint8_t mesh_channel_hash(const char *channel_name, const uint8_t *key, size_t key_len);

/** The well-known default channel key ("AQ==", PSK index 1). */
extern const uint8_t mesh_default_key[16];

/**
 * Parse the header and, if channel_hash matches, decrypt and decode the payload.
 *
 * @param buf          Received frame. The payload part is decrypted in place.
 * @param len          Frame length.
 * @param channel_hash Hash of the channel we hold the key for.
 * @param key          AES-128 channel key.
 * @param out          Decoded packet.
 * @return 0 on success (header valid), negative if the frame is too short.
 */
int mesh_decode(uint8_t *buf, size_t len, uint8_t channel_hash, const uint8_t key[16],
		struct mesh_packet *out);

/** Payload of an outgoing packet (the Data protobuf, before encryption). */
struct mesh_tx_data {
	uint32_t portnum;
	const uint8_t *payload;
	size_t payload_len;
	bool want_response;
	uint32_t request_id;
};

/** Header fields of an outgoing packet. */
struct mesh_tx_header {
	uint32_t to;
	uint32_t from;
	uint32_t id;
	uint8_t hop_limit;      /* also used as hop_start */
	bool want_ack;
	uint8_t channel_hash;
	uint8_t relay_node;     /* last byte of our node number */
};

/**
 * Build a complete over-the-air frame: header + encrypted Data protobuf.
 * @return frame length, or a negative value if it does not fit.
 */
int mesh_encode(uint8_t *out, size_t out_size, const struct mesh_tx_header *hdr,
		const struct mesh_tx_data *data, const uint8_t key[16]);

/** Payload builders. Each returns the payload length (0 on overflow). */
size_t mesh_build_user(uint8_t *buf, size_t size, uint32_t node_num, const char *long_name,
		       const char *short_name, uint32_t hw_model, uint32_t role);
size_t mesh_build_position(uint8_t *buf, size_t size, int32_t lat_i, int32_t lon_i,
			   int32_t alt_m, bool has_alt);
size_t mesh_build_routing_ack(uint8_t *buf, size_t size);

/** Short name for a port number ("NODEINFO", "POSITION", ...). */
const char *mesh_port_name(uint32_t portnum);

/** Device role name ("CLIENT", "ROUTER", ...). */
const char *mesh_role_name(uint32_t role);

/** Hardware model name from meshtastic/mesh.proto HardwareModel. */
const char *mesh_hw_model_name(uint32_t hw_model);

#endif /* MESHTASTIC_H_ */
