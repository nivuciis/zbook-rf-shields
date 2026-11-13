/**
 * @file main.c
 *
 * @brief Meshtastic node for the ZBook + RFM95W.
 *
 * Joins the Meshtastic primary channel (LongFast, default key) as a CLIENT
 * node: announces itself (NODEINFO), relays packets for its neighbours
 * (managed flooding), answers ACKs and NODEINFO requests, and sends and
 * receives text messages. Everything is driven from the Zephyr shell
 * ("mesh" commands) on the PROG USB serial port.
 *
 * @author Vinicius RM Carvalho
 * @date 2026-11-12
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "mesh.h"
#include "meshtastic.h"
#include "nodedb.h"
#include "radio.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define DEFAULT_REGION       "US"
#define DEFAULT_PRESET       "LongFast"
#define NODEINFO_PERIOD      K_HOURS(3)    /* Meshtastic default */
#define POSITION_PERIOD      K_MINUTES(30)

/* ------------------------------------------------------- periodic sends */

static void nodeinfo_work_fn(struct k_work *work);
static void position_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(nodeinfo_work, nodeinfo_work_fn);
static K_WORK_DELAYABLE_DEFINE(position_work, position_work_fn);

static void nodeinfo_work_fn(struct k_work *work)
{
	mesh_send_nodeinfo(MESH_BROADCAST, false);
	k_work_reschedule(k_work_delayable_from_work(work), NODEINFO_PERIOD);
}

static void position_work_fn(struct k_work *work)
{
	if (mesh_has_position()) {
		mesh_send_position(MESH_BROADCAST);
	}
	k_work_reschedule(k_work_delayable_from_work(work), POSITION_PERIOD);
}

/* ---------------------------------------------------------------- helpers */

/* Degrees * 1e7 -> "-9.64980" without floating point. */
static void fmt_coord(char *buf, size_t size, int32_t v)
{
	uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;

	snprintk(buf, size, "%s%u.%05u", v < 0 ? "-" : "", a / 10000000U, (a % 10000000U) / 100U);
}

/* "-9.6498" -> -96498000 (degrees * 1e7) without floating point. */
static bool parse_coord(const char *s, int32_t *out)
{
	bool neg = false;
	int64_t ip = 0, frac = 0, scale = 10000000;

	if (*s == '-' || *s == '+') {
		neg = (*s == '-');
		s++;
	}
	if (*s < '0' || *s > '9') {
		return false;
	}
	while (*s >= '0' && *s <= '9') {
		ip = ip * 10 + (*s++ - '0');
	}
	if (*s == '.') {
		s++;
		while (*s >= '0' && *s <= '9') {
			if (scale > 1) {
				scale /= 10;
				frac += (*s - '0') * scale;
			}
			s++;
		}
	}
	if (*s != '\0' || ip > 180) {
		return false;
	}
	*out = (int32_t)((ip * 10000000 + frac) * (neg ? -1 : 1));
	return true;
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

/* Join argv[first..] with spaces (shell splits the message into words). */
static bool join_args(char *buf, size_t size, size_t argc, char **argv, size_t first)
{
	size_t len = 0;

	buf[0] = '\0';
	for (size_t i = first; i < argc; i++) {
		size_t n = strlen(argv[i]);

		if (len + n + 2 > size) {
			return false;
		}
		if (len) {
			buf[len++] = ' ';
		}
		memcpy(&buf[len], argv[i], n);
		len += n;
		buf[len] = '\0';
	}
	return len > 0;
}

/* ----------------------------------------------------------------- shell */

static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	const struct mesh_identity *me = mesh_identity();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Node      : !%08x", me->num);
	shell_print(sh, "Name      : %s (%s)", me->long_name, me->short_name);
	shell_print(sh, "Hardware  : PRIVATE_HW, role CLIENT");
	shell_print(sh, "Relay     : %s", mesh_relay_enabled() ? "on" : "off");
	return 0;
}

static int cmd_name(const struct shell *sh, size_t argc, char **argv)
{
	if (strlen(argv[1]) > 39 || (argc > 2 && strlen(argv[2]) > 4)) {
		shell_error(sh, "Long name up to 39 characters, short name up to 4");
		return -EINVAL;
	}
	mesh_set_name(argv[1], argc > 2 ? argv[2] : NULL);
	shell_print(sh, "Name set (not saved across resets). Announcing...");
	return mesh_send_nodeinfo(MESH_BROADCAST, false);
}

static int cmd_announce(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Sending NODEINFO and asking neighbours for theirs...");
	return mesh_send_nodeinfo(MESH_BROADCAST, true);
}

static int cmd_send(const struct shell *sh, size_t argc, char **argv)
{
	char text[201];
	uint32_t id;
	int ret;

	if (!join_args(text, sizeof(text), argc, argv, 1)) {
		shell_error(sh, "Message too long (200 characters max)");
		return -EINVAL;
	}
	ret = mesh_send_text(MESH_BROADCAST, text, false, &id);
	if (ret == 0) {
		shell_print(sh, "Queued to everyone on %s (id %08x)", radio_get()->preset->name, id);
	}
	return ret;
}

static int cmd_dm(const struct shell *sh, size_t argc, char **argv)
{
	char text[201];
	uint32_t to, id;
	int ret;

	if (!parse_node_id(argv[1], &to)) {
		shell_error(sh, "Invalid node id: %s (use !a1b2c3d4)", argv[1]);
		return -EINVAL;
	}
	if (!join_args(text, sizeof(text), argc, argv, 2)) {
		shell_error(sh, "Message too long (200 characters max)");
		return -EINVAL;
	}
	ret = mesh_send_text(to, text, true, &id);
	if (ret == 0) {
		shell_print(sh, "Queued to !%08x (id %08x), waiting for ACK...", to, id);
	}
	return ret;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	int32_t lat, lon, alt = 0;
	char a[16], b[16];

	if (argc == 2 && strcmp(argv[1], "off") == 0) {
		mesh_clear_position();
		shell_print(sh, "Position cleared.");
		return 0;
	}
	if (argc < 3 || !parse_coord(argv[1], &lat) || !parse_coord(argv[2], &lon)) {
		shell_error(sh, "Use: mesh pos <lat> <lon> [alt_m]   e.g. mesh pos -9.6498 -35.7089 10");
		return -EINVAL;
	}
	if (argc > 3) {
		alt = atoi(argv[3]);
	}
	mesh_set_position(lat, lon, alt);
	fmt_coord(a, sizeof(a), lat);
	fmt_coord(b, sizeof(b), lon);
	shell_print(sh, "Position %s, %s, %d m. Sending...", a, b, alt);
	return mesh_send_position(MESH_BROADCAST);
}

static int cmd_nodes(const struct shell *sh, size_t argc, char **argv)
{
	static struct node_entry list[NODEDB_MAX_NODES];
	int n = nodedb_snapshot(list, ARRAY_SIZE(list));
	char ago[8], rssi[8], snr[8], hops[4];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (n == 0) {
		shell_print(sh, "No nodes heard yet. Try 'mesh announce'.");
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

	ARG_UNUSED(argc);
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
	struct radio_stats rs;
	struct mesh_stats ms;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	radio_get_stats(&rs);
	mesh_get_stats(&ms);

	shell_print(sh, "Region   : %s", s->region->name);
	shell_print(sh, "Preset   : %s (SF%u, BW %u kHz, CR 4/%u)", s->preset->name,
		    s->preset->sf, s->preset->bw_khz, s->preset->cr);
	shell_print(sh, "Frequency: %u.%03u MHz (slot %u of %u)", s->freq_hz / 1000000U,
		    (s->freq_hz / 1000U) % 1000U, s->slot + 1, s->num_slots);
	shell_print(sh, "Channel  : hash 0x%02x, default key", s->channel_hash);
	shell_print(sh, "TX power : %u dBm", s->tx_power_dbm);
	shell_print(sh, "DIO0     : %s (%u by interrupt, %u found by polling)",
		    !rs.dio0_configured ? "not configured"
		    : rs.dio0_hits ? "working" : "no edge seen yet",
		    rs.dio0_hits, rs.dio0_misses);
	shell_print(sh, "Radio    : %u RX, %u CRC errors, %u TX (%u ms on air)", rs.rx_frames,
		    rs.crc_errors, rs.tx_frames, rs.tx_airtime_ms);
	shell_print(sh, "Mesh     : %u new, %u decrypted, %u other channel, %u duplicates",
		    ms.received, ms.decrypted, ms.other_channel, ms.duplicates);
	shell_print(sh, "Routing  : %u sent, %u relayed, %u relays cancelled", ms.sent,
		    ms.relayed, ms.relays_cancelled);
	return 0;
}

static int cmd_region(const struct shell *sh, size_t argc, char **argv)
{
	const struct mesh_region *r = radio_find_region(argv[1]);

	ARG_UNUSED(argc);
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

	ARG_UNUSED(argc);
	if (p == NULL) {
		shell_error(sh, "Unknown preset. Options:");
		for (size_t i = 0; i < mesh_preset_count; i++) {
			shell_print(sh, "  %s", mesh_presets[i].name);
		}
		return -EINVAL;
	}
	return radio_set(radio_get()->region, p);
}

static int cmd_txpower(const struct shell *sh, size_t argc, char **argv)
{
	int dbm = atoi(argv[1]);

	ARG_UNUSED(argc);
	if (radio_set_tx_power((uint8_t)CLAMP(dbm, 0, 255)) < 0) {
		shell_error(sh, "Use 2..17 dBm");
		return -EINVAL;
	}
	shell_print(sh, "TX power %d dBm", dbm);
	return 0;
}

static int on_off(const struct shell *sh, const char *arg, bool *out)
{
	if (strcmp(arg, "on") == 0) {
		*out = true;
	} else if (strcmp(arg, "off") == 0) {
		*out = false;
	} else {
		shell_error(sh, "Use on or off");
		return -EINVAL;
	}
	return 0;
}

static int cmd_relay(const struct shell *sh, size_t argc, char **argv)
{
	bool on;

	ARG_UNUSED(argc);
	if (on_off(sh, argv[1], &on) == 0) {
		mesh_set_relay(on);
		return 0;
	}
	return -EINVAL;
}

static int cmd_live(const struct shell *sh, size_t argc, char **argv)
{
	bool on;

	ARG_UNUSED(argc);
	if (on_off(sh, argv[1], &on) == 0) {
		mesh_set_live(on);
		return 0;
	}
	return -EINVAL;
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
	diag_line(sh, "PaConfig", d.pa_config, want.pa_config, 2);

	shell_print(sh, "Channel RSSI for %d s:", seconds);
	for (int i = 0; i < seconds; i++) {
		if (radio_read_diag(&d) == 0) {
			shell_print(sh, "  %2d s: %4d dBm%s", i + 1, d.rssi_now,
				    (d.modem_stat & 0x01) ? "  (LoRa signal detected)" : "");
		}
		k_sleep(K_SECONDS(1));
	}
	return 0;
}

static int cmd_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	nodedb_clear();
	shell_print(sh, "Node table cleared.");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mesh_cmds,
	SHELL_CMD(id, NULL, "This node's ID and name", cmd_id),
	SHELL_CMD_ARG(name, NULL, "Set name: mesh name <long> [short]", cmd_name, 2, 1),
	SHELL_CMD(announce, NULL, "Send NODEINFO and ask neighbours for theirs", cmd_announce),
	SHELL_CMD_ARG(send, NULL, "Message to everyone: mesh send <text>", cmd_send, 2, 30),
	SHELL_CMD_ARG(dm, NULL, "Direct message with ACK: mesh dm !id <text>", cmd_dm, 3, 30),
	SHELL_CMD_ARG(pos, NULL, "Fixed position: mesh pos <lat> <lon> [alt] | off", cmd_pos, 2, 2),
	SHELL_CMD(nodes, NULL, "List the nodes heard", cmd_nodes),
	SHELL_CMD_ARG(node, NULL, "Details of one node: mesh node !a1b2c3d4", cmd_node, 2, 0),
	SHELL_CMD(radio, NULL, "Radio settings and counters", cmd_radio),
	SHELL_CMD_ARG(region, NULL, "Set region: US | ANZ | BR_902", cmd_region, 2, 0),
	SHELL_CMD_ARG(preset, NULL, "Set modem preset: LongFast, MediumFast, ...", cmd_preset, 2, 0),
	SHELL_CMD_ARG(txpower, NULL, "TX power in dBm: mesh txpower 2..17", cmd_txpower, 2, 0),
	SHELL_CMD_ARG(relay, NULL, "Relay other nodes' packets: mesh relay on|off", cmd_relay, 2, 0),
	SHELL_CMD_ARG(live, NULL, "Print each packet: mesh live on|off", cmd_live, 2, 0),
	SHELL_CMD_ARG(diag, NULL, "Check radio registers and RSSI: mesh diag [s]", cmd_diag, 1, 1),
	SHELL_CMD(clear, NULL, "Clear the node table", cmd_clear),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mesh, &mesh_cmds, "Meshtastic node", NULL);

/* ------------------------------------------------------------------ main */

int main(void)
{
	int ret = radio_init(radio_find_region(DEFAULT_REGION), radio_find_preset(DEFAULT_PRESET));

	if (ret < 0) {
		LOG_ERR("Radio init failed (%d). Check the SPI wiring.", ret);
		return 0;
	}

	mesh_start();
	k_work_schedule(&nodeinfo_work, K_SECONDS(15));
	k_work_schedule(&position_work, K_SECONDS(20));

	LOG_INF("Meshtastic node ready. Type 'mesh help'. Antenna must be connected!");
	return 0;
}
