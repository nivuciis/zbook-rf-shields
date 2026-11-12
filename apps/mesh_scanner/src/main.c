/**
 * @file main.c
 *
 * @brief Meshtastic node scanner for the ZBook + RFM95W (receive only).
 *
 * Listens on a Meshtastic channel (default: LongFast, default key), decodes
 * the packets it can and keeps a table of the nodes heard. The table and the
 * radio settings are available through the Zephyr shell ("mesh" commands).
 * Nothing is ever transmitted.
 *
 * @author Vinicius RM Carvalho
 * @date 2026-11-11
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "meshtastic.h"
#include "nodedb.h"
#include "radio.h"

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_INF);

#define DEFAULT_REGION "US"
#define DEFAULT_PRESET "LongFast"

static bool live_output = true;
static uint32_t stat_frames, stat_decrypted, stat_other_channel, stat_bad;

/* ---------------------------------------------------------------- helpers */

/* Degrees * 1e7 -> "-9.64980" without floating point. */
static void fmt_coord(char *buf, size_t size, int32_t v)
{
	uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;

	snprintk(buf, size, "%s%u.%05u", v < 0 ? "-" : "", a / 10000000U, (a % 10000000U) / 100U);
}

static void fmt_ago(char *buf, size_t size, int64_t since_ms)
{
	int s = (int)((k_uptime_get() - since_ms) / 1000);

	if (s < 60) {
		snprintk(buf, size, "%ds", s);
	} else if (s < 3600) {
		snprintk(buf, size, "%dm", s / 60);
	} else {
		snprintk(buf, size, "%dh", s / 3600);
	}
}

static bool parse_node_id(const char *s, uint32_t *num)
{
	char *end;

	if (*s == '!') {
		s++;
	}
	*num = strtoul(s, &end, 16);
	return *s != '\0' && *end == '\0';
}

/* ------------------------------------------------------------ RX thread */

static void print_packet(const struct mesh_packet *p, const struct radio_frame *f)
{
	struct node_entry n;
	const char *name = "";
	int hops = p->hop_start >= p->hop_limit && p->hop_start ? p->hop_start - p->hop_limit : -1;
	char coord_lat[16], coord_lon[16];

	if (nodedb_get(p->from, &n) && n.has_user) {
		name = n.short_name;
	}

	if (!p->decrypted) {
		LOG_INF("!%08x %-4s ch=0x%02x (other channel/encrypted) hops=%d %d dBm %d dB",
			p->from, name, p->channel_hash, hops, f->rssi, f->snr);
		return;
	}

	switch (p->kind) {
	case MESH_INFO_USER:
		LOG_INF("!%08x %-4s NODEINFO \"%s\" %s %s hops=%d %d dBm %d dB", p->from, name,
			p->info.user.long_name, mesh_hw_model_name(p->info.user.hw_model),
			mesh_role_name(p->info.user.role), hops, f->rssi, f->snr);
		break;
	case MESH_INFO_POSITION:
		fmt_coord(coord_lat, sizeof(coord_lat), p->info.pos.lat_i);
		fmt_coord(coord_lon, sizeof(coord_lon), p->info.pos.lon_i);
		LOG_INF("!%08x %-4s POSITION %s, %s hops=%d %d dBm %d dB", p->from, name,
			coord_lat, coord_lon, hops, f->rssi, f->snr);
		break;
	case MESH_INFO_TEXT:
		LOG_INF("!%08x %-4s TEXT \"%s\" hops=%d %d dBm %d dB", p->from, name,
			p->info.text, hops, f->rssi, f->snr);
		break;
	case MESH_INFO_BATTERY:
		LOG_INF("!%08x %-4s TELEMETRY battery=%u%% hops=%d %d dBm %d dB", p->from, name,
			p->info.battery_level, hops, f->rssi, f->snr);
		break;
	default:
		LOG_INF("!%08x %-4s %s hops=%d %d dBm %d dB", p->from, name,
			mesh_port_name(p->portnum), hops, f->rssi, f->snr);
		break;
	}
}

/* Raw test mode (sync word other than Meshtastic's): show the frame as is. */
static void print_raw_frame(const struct radio_frame *f)
{
	char hex[3 * 32 + 1];
	char text[32 + 1];
	size_t n = MIN(f->len, 32U);

	for (size_t i = 0; i < n; i++) {
		snprintk(&hex[3 * i], 4, "%02x ", f->data[i]);
		text[i] = (f->data[i] >= 0x20 && f->data[i] < 0x7F) ? f->data[i] : '.';
	}
	hex[3 * n] = '\0';
	text[n] = '\0';

	LOG_INF("RAW %u bytes, %d dBm, SNR %d dB: %s| %s%s", f->len, f->rssi, f->snr, hex, text,
		f->len > n ? " ..." : "");
}

static void rx_thread(void *a, void *b, void *c)
{
	static struct radio_frame frame;
	static struct mesh_packet pkt;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		if (radio_get_frame(&frame, -1) != 0) {
			continue;
		}
		stat_frames++;

		if (radio_get()->sync_word != MESHTASTIC_SYNC_WORD) {
			print_raw_frame(&frame);
			continue;
		}

		if (mesh_decode(frame.data, frame.len, radio_get()->channel_hash, mesh_default_key,
				&pkt) < 0) {
			stat_bad++;
			if (live_output) {
				LOG_WRN("Frame too short (%u bytes), not Meshtastic", frame.len);
			}
			continue;
		}

		if (pkt.decrypted) {
			stat_decrypted++;
		} else {
			stat_other_channel++;
		}

		if (nodedb_update(&pkt, frame.rssi, frame.snr)) {
			LOG_INF("New node: !%08x", pkt.from);
		}
		if (live_output) {
			print_packet(&pkt, &frame);
		}
	}
}

K_THREAD_DEFINE(rx_tid, 4096, rx_thread, NULL, NULL, NULL, 7, 0, 0);

/* --------------------------------------------------------------- shell */

static int cmd_nodes(const struct shell *sh, size_t argc, char **argv)
{
	static struct node_entry list[NODEDB_MAX_NODES];
	int n = nodedb_snapshot(list, ARRAY_SIZE(list));
	char ago[8], rssi[8], snr[8], hops[4];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (n == 0) {
		shell_print(sh, "No nodes heard yet.");
		return 0;
	}

	shell_print(sh, "%-10s %-5s %-24s %4s %5s %4s %5s %5s  %s", "ID", "Short", "Long name",
		    "Hops", "RSSI", "SNR", "Seen", "Pkts", "Hardware");
	for (int i = 0; i < n; i++) {
		const struct node_entry *e = &list[i];

		fmt_ago(ago, sizeof(ago), e->last_heard_ms);
		if (e->hops_away < 0) {
			strcpy(hops, "?");
		} else {
			snprintk(hops, sizeof(hops), "%d", e->hops_away);
		}
		if (e->has_signal) {
			snprintk(rssi, sizeof(rssi), "%d", e->rssi);
			snprintk(snr, sizeof(snr), "%d", e->snr);
		} else {
			strcpy(rssi, "-");
			strcpy(snr, "-");
		}
		shell_print(sh, "!%08x  %-5s %-24.24s %4s %5s %4s %5s %5u  %s", e->num,
			    e->has_user ? e->short_name : "?", e->has_user ? e->long_name : "",
			    hops, rssi, snr, ago, e->packets,
			    e->has_user ? mesh_hw_model_name(e->hw_model) : "");
	}
	shell_print(sh, "%d node(s). RSSI/SNR only for nodes heard directly (0 hops).", n);
	return 0;
}

static int cmd_node(const struct shell *sh, size_t argc, char **argv)
{
	struct node_entry e;
	uint32_t num;
	char ago[8], lat[16], lon[16];

	if (!parse_node_id(argv[1], &num)) {
		shell_error(sh, "Invalid node id: %s (use !a1b2c3d4)", argv[1]);
		return -EINVAL;
	}
	if (!nodedb_get(num, &e)) {
		shell_error(sh, "Node !%08x not heard yet", num);
		return -ENOENT;
	}

	fmt_ago(ago, sizeof(ago), e.last_heard_ms);
	shell_print(sh, "Node      : !%08x", e.num);
	if (e.has_user) {
		shell_print(sh, "Name      : %s (%s)", e.long_name, e.short_name);
		shell_print(sh, "Hardware  : %s", mesh_hw_model_name(e.hw_model));
		shell_print(sh, "Role      : %s", mesh_role_name(e.role));
	} else {
		shell_print(sh, "Name      : unknown (no NODEINFO heard yet)");
	}
	shell_print(sh, "Last heard: %s ago, %u packet(s), last port %s%s", ago, e.packets,
		    mesh_port_name(e.last_port), e.last_decrypted ? "" : " (not decrypted)");
	if (e.hops_away >= 0) {
		shell_print(sh, "Hops away : %d", e.hops_away);
	}
	if (e.has_signal) {
		shell_print(sh, "Signal    : %d dBm, SNR %d dB (direct)", e.rssi, e.snr);
	} else {
		shell_print(sh, "Signal    : only heard through relays (last relay ..%02x)",
			    e.last_relay);
	}
	if (e.has_pos) {
		fmt_coord(lat, sizeof(lat), e.lat_i);
		fmt_coord(lon, sizeof(lon), e.lon_i);
		shell_print(sh, "Position  : %s, %s, %d m", lat, lon, e.alt_m);
	}
	if (e.has_battery) {
		shell_print(sh, "Battery   : %u%%", e.battery_level);
	}
	return 0;
}

static int cmd_radio(const struct shell *sh, size_t argc, char **argv)
{
	const struct radio_state *s = radio_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Region   : %s", s->region->name);
	shell_print(sh, "Preset   : %s (SF%u, BW %u kHz, CR 4/%u)", s->preset->name,
		    s->preset->sf, s->preset->bw_khz, s->preset->cr);
	shell_print(sh, "Frequency: %u.%03u MHz (slot %u of %u)", s->freq_hz / 1000000U,
		    (s->freq_hz / 1000U) % 1000U, s->slot + 1, s->num_slots);
	shell_print(sh, "Channel  : hash 0x%02x, default key", s->channel_hash);
	shell_print(sh, "Sync word: 0x%02x%s", s->sync_word,
		    s->sync_word == MESHTASTIC_SYNC_WORD ? " (Meshtastic)" : " (raw test mode)");
	shell_print(sh, "Frames   : %u total, %u decrypted, %u other channel, %u invalid",
		    stat_frames, stat_decrypted, stat_other_channel, stat_bad);
	shell_print(sh, "CRC errors: %u (packets heard but corrupted)", radio_crc_errors());
	return 0;
}

static int cmd_region(const struct shell *sh, size_t argc, char **argv)
{
	const struct mesh_region *r = radio_find_region(argv[1]);

	if (r == NULL) {
		shell_error(sh, "Unknown region. Options:");
		for (size_t i = 0; i < mesh_region_count; i++) {
			shell_print(sh, "  %s", mesh_regions[i].name);
		}
		return -EINVAL;
	}
	return radio_set(r, radio_get()->preset);
}

static int cmd_preset(const struct shell *sh, size_t argc, char **argv)
{
	const struct mesh_preset *p = radio_find_preset(argv[1]);

	if (p == NULL) {
		shell_error(sh, "Unknown preset. Options:");
		for (size_t i = 0; i < mesh_preset_count; i++) {
			shell_print(sh, "  %s", mesh_presets[i].name);
		}
		return -EINVAL;
	}
	return radio_set(radio_get()->region, p);
}

static int cmd_live(const struct shell *sh, size_t argc, char **argv)
{
	if (strcmp(argv[1], "on") == 0) {
		live_output = true;
	} else if (strcmp(argv[1], "off") == 0) {
		live_output = false;
	} else {
		shell_error(sh, "Use: mesh live on|off");
		return -EINVAL;
	}
	return 0;
}

static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
	const struct mesh_preset *preset = radio_get()->preset;
	const struct mesh_region *best = radio_get()->region;
	uint32_t best_count = 0;
	int seconds = argc > 1 ? atoi(argv[1]) : 60;

	if (seconds <= 0) {
		shell_error(sh, "Invalid duration");
		return -EINVAL;
	}

	shell_print(sh, "Listening %d s on each region with %s...", seconds, preset->name);
	for (size_t i = 0; i < mesh_region_count; i++) {
		uint32_t before;

		if (radio_set(&mesh_regions[i], preset) < 0) {
			continue;
		}
		before = radio_rx_count();
		k_sleep(K_SECONDS(seconds));

		uint32_t heard = radio_rx_count() - before;

		shell_print(sh, "  %-7s %u.%03u MHz: %u frame(s)", mesh_regions[i].name,
			    radio_get()->freq_hz / 1000000U, (radio_get()->freq_hz / 1000U) % 1000U,
			    heard);
		if (heard > best_count) {
			best_count = heard;
			best = &mesh_regions[i];
		}
	}

	shell_print(sh, best_count ? "Staying on %s." : "Nothing heard; back to %s.", best->name);
	return radio_set(best, preset);
}

static void diag_line(const struct shell *sh, const char *name, uint32_t got, uint32_t want,
		      int width)
{
	shell_print(sh, "  %-14s 0x%0*x  (expected 0x%0*x)  %s", name, width, got, width, want,
		    got == want ? "OK" : "<-- MISMATCH");
}

static int cmd_diag(const struct shell *sh, size_t argc, char **argv)
{
	struct radio_diag d, want;
	int seconds = argc > 1 ? atoi(argv[1]) : 10;

	if (radio_read_diag(&d) < 0) {
		shell_error(sh, "SPI error while reading the radio");
		return -EIO;
	}
	radio_expected_diag(&want);

	shell_print(sh, "SX1276 registers (version 0x%02x):", d.version);
	diag_line(sh, "OpMode", d.op_mode, want.op_mode, 2);
	diag_line(sh, "Frequency", d.frf, want.frf, 6);
	diag_line(sh, "ModemConfig1", d.modem_config_1, want.modem_config_1, 2);
	diag_line(sh, "ModemConfig2", d.modem_config_2, want.modem_config_2, 2);
	diag_line(sh, "ModemConfig3", d.modem_config_3, want.modem_config_3, 2);
	diag_line(sh, "SyncWord", d.sync_word, want.sync_word, 2);
	diag_line(sh, "Preamble", d.preamble, want.preamble, 4);
	shell_print(sh, "Since RX started: %u valid header(s), %u packet(s) with good CRC",
		    d.valid_headers, d.valid_packets);

	shell_print(sh, "Channel RSSI for %d s (noise floor is usually -125..-105 dBm):", seconds);
	for (int i = 0; i < seconds; i++) {
		if (radio_read_diag(&d) == 0) {
			shell_print(sh, "  %2d s: %4d dBm%s", i + 1, d.rssi_now,
				    (d.modem_stat & 0x01) ? "  (LoRa signal detected)" : "");
		}
		k_sleep(K_SECONDS(1));
	}
	return 0;
}

static int cmd_sync(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long v = strtoul(argv[1], &end, 16);

	if (*argv[1] == '\0' || *end != '\0' || v == 0 || v > 0xFF) {
		shell_error(sh, "Use a hex byte: mesh sync 2b (Meshtastic) or mesh sync 12");
		return -EINVAL;
	}
	return radio_set_sync_word((uint8_t)v);
}

static int cmd_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	nodedb_clear();
	stat_frames = stat_decrypted = stat_other_channel = stat_bad = 0;
	shell_print(sh, "Node table cleared.");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mesh_cmds,
	SHELL_CMD(nodes, NULL, "List the nodes heard", cmd_nodes),
	SHELL_CMD_ARG(node, NULL, "Details of one node: mesh node !a1b2c3d4", cmd_node, 2, 0),
	SHELL_CMD(radio, NULL, "Radio settings and counters", cmd_radio),
	SHELL_CMD_ARG(region, NULL, "Set region: US | ANZ | BR_902", cmd_region, 2, 0),
	SHELL_CMD_ARG(preset, NULL, "Set modem preset: LongFast, MediumFast, ...", cmd_preset, 2, 0),
	SHELL_CMD_ARG(live, NULL, "Print each packet: mesh live on|off", cmd_live, 2, 0),
	SHELL_CMD_ARG(scan, NULL, "Try every region: mesh scan [seconds per region]", cmd_scan, 1, 1),
	SHELL_CMD_ARG(diag, NULL, "Check radio registers and RSSI: mesh diag [seconds]", cmd_diag, 1, 1),
	SHELL_CMD_ARG(sync, NULL, "Sync word in hex: 2b = Meshtastic, other = raw test mode",
		      cmd_sync, 2, 0),
	SHELL_CMD(clear, NULL, "Clear the node table and counters", cmd_clear),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mesh, &mesh_cmds, "Meshtastic scanner", NULL);

/* ---------------------------------------------------------------- main */

int main(void)
{
	int ret = radio_init(radio_find_region(DEFAULT_REGION), radio_find_preset(DEFAULT_PRESET));

	if (ret < 0) {
		LOG_ERR("Radio init failed (%d). Check the SPI wiring.", ret);
		return 0;
	}

	LOG_INF("Meshtastic scanner ready (receive only). Type 'mesh help'.");
	return 0;
}
