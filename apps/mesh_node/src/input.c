/**
 * @file input.c
 * @brief Board buttons and rotary encoder -> ui_input_chan.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * The ZBook devicetree already exposes the hardware through the Zephyr input
 * subsystem: gpio-keys for BTN0..BTN3 (INPUT_BTN_0..3) and the encoder push
 * button (INPUT_KEY_ENTER), gpio-qdec for the encoder (INPUT_REL_WHEEL).
 * This module only translates those events into UI events.
 */
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "events.h"

LOG_MODULE_REGISTER(ui_input, LOG_LEVEL_INF);

static void on_input(struct input_event *evt, void *user_data)
{
	struct ui_input in = {0};

	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_WHEEL) {
		int v = CLAMP(evt->value, -100, 100);

		in.rotate = true;
		in.delta = (int8_t)(IS_ENABLED(CONFIG_APP_UI_ENCODER_REVERSE) ? -v : v);
	} else if (evt->type == INPUT_EV_KEY && evt->value == 1) { /* press only */
		switch (evt->code) {
		case INPUT_BTN_0:
			in.key = UI_KEY_BACK;
			break;
		case INPUT_BTN_1:
		case INPUT_KEY_ENTER:
			in.key = UI_KEY_OK;
			break;
		case INPUT_BTN_2:
			in.key = UI_KEY_ACTION;
			break;
		case INPUT_BTN_3:
			in.key = UI_KEY_HOME;
			break;
		default:
			return;
		}
	} else {
		return;
	}

	if (zbus_chan_pub(&ui_input_chan, &in, K_MSEC(10)) != 0) {
		LOG_WRN("UI busy, input dropped");
	}
}

INPUT_CALLBACK_DEFINE(NULL, on_input, NULL);
