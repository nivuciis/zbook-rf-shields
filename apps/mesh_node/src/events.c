/**
 * @file events.c
 * @brief zbus channel definitions (observers are added where they live).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "events.h"

ZBUS_CHAN_DEFINE(ui_input_chan, struct ui_input, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

ZBUS_CHAN_DEFINE(mesh_event_chan, struct mesh_event, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));
