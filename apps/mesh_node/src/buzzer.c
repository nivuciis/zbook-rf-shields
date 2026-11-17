/**
 * @file buzzer.c
 * @brief Notification sounds on the ZBook piezo buzzer.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * A second observer of mesh_event_chan, independent of the UI: it plays a
 * short tune whenever a text message arrives (a different one for direct
 * messages). The buzzer is the board's "pwm-buzzer" (GPIO32, PWM slice 8),
 * driven with a 50 % duty cycle at the note frequency.
 */
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "buzzer.h"
#include "events.h"
#include "meshtastic.h"

LOG_MODULE_REGISTER(buzzer, LOG_LEVEL_INF);

struct note {
	uint16_t hz;  /* 0 = silence */
	uint16_t ms;
};

/* Channel message: two quick notes. */
static const struct note tune_channel[] = {
	{1760, 70}, {0, 50}, {2349, 110},
};

/* Direct message: three rising notes, easier to tell apart. */
static const struct note tune_direct[] = {
	{1568, 70}, {0, 30}, {2093, 70}, {0, 30}, {2637, 140},
};

static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(pwm_buzzer));
static bool enabled = true;

static void play(const struct note *tune, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (tune[i].hz == 0) {
			pwm_set_pulse_dt(&buzzer, 0);
		} else {
			uint32_t period = PWM_HZ(tune[i].hz);

			pwm_set_dt(&buzzer, period, period / 2);
		}
		k_msleep(tune[i].ms);
	}
	pwm_set_pulse_dt(&buzzer, 0);
}

void buzzer_set_enabled(bool on)
{
	enabled = on;
}

bool buzzer_enabled(void)
{
	return enabled;
}

ZBUS_MSG_SUBSCRIBER_DEFINE(buzzer_sub);
ZBUS_CHAN_ADD_OBS(mesh_event_chan, buzzer_sub, 4);

static void buzzer_thread(void *a, void *b, void *c)
{
	const struct zbus_channel *chan;
	struct mesh_event ev;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	if (!pwm_is_ready_dt(&buzzer)) {
		LOG_ERR("Buzzer PWM not ready");
		return;
	}
	pwm_set_pulse_dt(&buzzer, 0);

	while (zbus_sub_wait_msg(&buzzer_sub, &chan, &ev, K_FOREVER) == 0) {
		if (!enabled || ev.type != MESH_EV_TEXT) {
			continue;
		}
		if (ev.to == MESH_BROADCAST) {
			play(tune_channel, ARRAY_SIZE(tune_channel));
		} else {
			play(tune_direct, ARRAY_SIZE(tune_direct));
		}
	}
}

K_THREAD_DEFINE(buzzer_tid, 1536, buzzer_thread, NULL, NULL, NULL, 12, 0, 0);
