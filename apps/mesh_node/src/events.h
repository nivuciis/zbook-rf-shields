/**
 * @file events.h
 * @brief zbus channels between the input devices, the mesh and the UI.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 *   input.c --ui_input_chan-->  ui.c   (buttons and rotary encoder)
 *   mesh.c  --mesh_event_chan-> ui.c   (messages, nodes, ACKs, sends)
 *
 * Both channels are observed by a message subscriber in ui.c, so the UI
 * receives every event in order, even while it is busy refreshing the OLED.
 */
#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/zbus/zbus.h>

/** The four board buttons (BTN0..BTN3) plus the encoder push button. */
enum ui_key {
	UI_KEY_BACK,    /* BTN0 */
	UI_KEY_OK,      /* BTN1 or encoder press */
	UI_KEY_ACTION,  /* BTN2: context action of the screen */
	UI_KEY_HOME,    /* BTN3 */
};

struct ui_input {
	bool rotate;    /* true: encoder turned by `delta` detents */
	int8_t delta;
	uint8_t key;    /* enum ui_key, when !rotate */
};

enum mesh_event_type {
	MESH_EV_TEXT,   /* text message for us or for everyone */
	MESH_EV_NODE,   /* node added or updated in the node table */
	MESH_EV_ACK,    /* ACK/NAK for one of our direct messages */
	MESH_EV_SENT,   /* one of our packets went out on the air */
};

#define MESH_EV_TEXT_MAX 200

struct mesh_event {
	uint8_t type;   /* enum mesh_event_type */
	bool ok;        /* MESH_EV_ACK: delivered */
	int8_t snr;
	int16_t rssi;
	uint32_t from;
	uint32_t to;
	uint32_t id;
	char text[MESH_EV_TEXT_MAX + 1];
};

ZBUS_CHAN_DECLARE(ui_input_chan, mesh_event_chan);

#endif /* EVENTS_H_ */
