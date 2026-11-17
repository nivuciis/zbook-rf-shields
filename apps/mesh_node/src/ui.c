/**
 * @file ui.c
 * @brief OLED user interface driven by the board buttons and rotary encoder.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Controls (the same on every screen):
 *   encoder turn   move the selection / change the value / scroll
 *   encoder press  OK (same as BTN1)
 *   BTN0           back (in the text editor: delete a character)
 *   BTN1           OK / select
 *   BTN2           action of the screen, shown in the footer
 *   BTN3           home
 *
 * The UI runs in its own thread. It receives every input and mesh event
 * through a zbus message subscriber, keeps a stack of screens, and redraws
 * the 1-bpp framebuffer (gfx.c) only when something changed.
 */
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "buzzer.h"
#include "events.h"
#include "gfx.h"
#include "mesh.h"
#include "meshtastic.h"
#include "nodedb.h"
#include "radio.h"
#include "ui.h"
#include "ui_draw.h"

LOG_MODULE_REGISTER(ui, LOG_LEVEL_INF);

#define STACK_DEPTH    8
#define HISTORY_LEN    16
#define POPUP_MS       4000
#define TOAST_MS       1500
#define TICK_MS        500

/* ================================================================ data */

struct msg_entry {
	uint32_t from;
	uint32_t to;
	int16_t rssi;
	int8_t snr;
	bool read;
	int64_t at_ms;
	char text[MESH_EV_TEXT_MAX + 1];
};

static struct msg_entry history[HISTORY_LEN];
static int history_count;          /* entries in use */
static int history_head;           /* next slot to write */

static struct node_entry nodes[NODEDB_MAX_NODES];
static int node_count;

static uint32_t compose_to = MESH_BROADCAST;
static uint32_t pending_dm_id;

static int32_t edit_lat, edit_lon, edit_alt; /* lat/lon in 1e-5 degrees */

static int screen_timeout_idx = 1;
static const int screen_timeouts_s[] = {30, 120, 300, 0}; /* 0 = never */

/* ============================================================== screens */

enum kind {
	K_HOME,
	K_LIST,
	K_TEXT,
	K_EDIT_TEXT,
	K_EDIT_NUM,
	K_CONFIRM,
};

/** A list screen: title, items and what OK / BTN2 do. */
struct list_src {
	const char *title;
	const char *footer;
	int (*count)(void);
	void (*label)(int i, char *buf, size_t size, void *ctx);
	void (*select)(int i);
	void (*action)(int i);     /* BTN2, optional */
	void (*refresh)(void);     /* reload data before drawing, optional */
};

/** A scrollable text screen. */
struct text_src {
	const char *title;
	const char *footer;
	void (*build)(char *buf, size_t size, uint32_t arg);
	void (*action)(uint32_t arg); /* BTN2, optional */
};

struct frame {
	enum kind kind;
	const struct list_src *list;
	const struct text_src *text;
	int sel;
	int top;
	int scroll;
	uint32_t arg;
};

static struct frame stack[STACK_DEPTH];
static int depth;

static struct frame *cur(void)
{
	return &stack[depth - 1];
}

static void push(struct frame f)
{
	if (depth < STACK_DEPTH) {
		stack[depth++] = f;
	}
}

static void pop(void)
{
	if (depth > 1) {
		depth--;
	}
}

static void go_home(void)
{
	depth = 1;
}

static void push_list(const struct list_src *src)
{
	push((struct frame){.kind = K_LIST, .list = src});
}

static void push_text(const struct text_src *src, uint32_t arg)
{
	push((struct frame){.kind = K_TEXT, .text = src, .arg = arg});
}

/* ======================================================= popups & toasts */

static char popup_title[24];
static char popup_body[MESH_EV_TEXT_MAX + 1];
static int64_t popup_until;

static void popup(const char *title, const char *body, int ms)
{
	snprintf(popup_title, sizeof(popup_title), "%s", title);
	snprintf(popup_body, sizeof(popup_body), "%s", body ? body : "");
	popup_until = k_uptime_get() + ms;
}

static void toast(const char *text)
{
	popup(text, "", TOAST_MS);
}

/* ============================================================== helpers */

static const char *node_name(uint32_t num, char *buf, size_t size)
{
	struct node_entry n;

	if (num == MESH_BROADCAST) {
		snprintf(buf, size, "All");
	} else if (nodedb_get(num, &n) && n.has_user) {
		snprintf(buf, size, "%s", n.short_name);
	} else {
		snprintf(buf, size, "!%08x", num);
	}
	return buf;
}

static void fmt_ago(char *buf, size_t size, int64_t since_ms)
{
	int s = (int)((k_uptime_get() - since_ms) / 1000);

	if (s < 60) {
		snprintf(buf, size, "%ds", s);
	} else if (s < 3600) {
		snprintf(buf, size, "%dm", s / 60);
	} else {
		snprintf(buf, size, "%dh", s / 3600);
	}
}

/* value in units of 10^-decimals, e.g. -965000 with 5 decimals -> "-9.65000" */
static void fmt_fixed(char *buf, size_t size, int32_t v, int decimals)
{
	uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
	uint32_t div = 1;

	for (int i = 0; i < decimals; i++) {
		div *= 10;
	}
	if (decimals == 0) {
		snprintf(buf, size, "%s%u", v < 0 ? "-" : "", a);
	} else {
		snprintf(buf, size, "%s%u.%0*u", v < 0 ? "-" : "", a / div, decimals, a % div);
	}
}

static int unread_count(void)
{
	int n = 0;

	for (int i = 0; i < history_count; i++) {
		if (!history[i].read) {
			n++;
		}
	}
	return n;
}

/* i = 0 is the most recent message */
static struct msg_entry *history_get(int i)
{
	int idx = (history_head - 1 - i + HISTORY_LEN) % HISTORY_LEN;

	return &history[idx];
}

static void reload_nodes(void)
{
	node_count = nodedb_snapshot(nodes, ARRAY_SIZE(nodes));
}

static void send_text_to(uint32_t to, const char *text)
{
	uint32_t id;
	bool dm = to != MESH_BROADCAST;
	int ret = mesh_send_text(to, text, dm, &id);

	if (ret == 0) {
		if (dm) {
			pending_dm_id = id;
		}
		toast(dm ? "Sending..." : "Sent to all");
	} else {
		toast("Send failed");
	}
}

/* ======================================================= text keyboard */

/*
 * On-screen keyboard, phone style: three rows of ten keys plus a row of
 * special keys. Turning the encoder moves along the row, BTN2 jumps to the
 * next row, OK types the key, BTN0 deletes the last character.
 */

#define KB_COLS     10
#define KB_ROWS     4      /* 3 character rows + 1 special row */
#define KB_SPECIAL  3      /* index of the special row */
#define KB_TEXT_ROW 1      /* screen rows 1..2 show the text */
#define KB_KEY_ROW  3      /* screen rows 3..5 character keys, 6 specials */
#define KB_KEY_W    12
#define KB_X0       4

enum kb_layout { KB_LOWER, KB_UPPER, KB_NUM, KB_ACCENT, KB_LAYOUTS };

/* Each layout: 3 rows x 10 keys, one UTF-8 character per key. */
static const char *const kb_keys[KB_LAYOUTS][3][KB_COLS] = {
	[KB_LOWER] = {
		{"q", "w", "e", "r", "t", "y", "u", "i", "o", "p"},
		{"a", "s", "d", "f", "g", "h", "j", "k", "l", "\xc3\xa7"},
		{"z", "x", "c", "v", "b", "n", "m", ",", ".", "?"},
	},
	[KB_UPPER] = {
		{"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
		{"A", "S", "D", "F", "G", "H", "J", "K", "L", "\xc3\x87"},
		{"Z", "X", "C", "V", "B", "N", "M", "!", ";", ":"},
	},
	[KB_NUM] = {
		{"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
		{"@", "#", "$", "%", "&", "*", "(", ")", "-", "+"},
		{"/", "'", "\"", "=", "_", "!", "?", ":", ";", ","},
	},
	[KB_ACCENT] = {
		{"\xc3\xa1", "\xc3\xa0", "\xc3\xa2", "\xc3\xa3", "\xc3\xa9", "\xc3\xaa", "\xc3\xad",
		 "\xc3\xb3", "\xc3\xb4", "\xc3\xb5"},
		{"\xc3\xba", "\xc3\xbc", "\xc3\xa7", "\xc3\x81", "\xc3\x80", "\xc3\x82", "\xc3\x83",
		 "\xc3\x89", "\xc3\x8a", "\xc3\x8d"},
		{"\xc3\x93", "\xc3\x94", "\xc3\x95", "\xc3\x9a", "\xc3\x87", "\xc3\xb1", "\xc2\xb0",
		 "\xc2\xbf", "\xc2\xa1", "."},
	},
};

enum kb_special { KS_SHIFT, KS_NUM, KS_ACCENT, KS_SPACE, KS_DEL, KS_DONE, KS_COUNT };

static const struct {
	const char *label;
	int x0, x1;
} kb_special_keys[KS_COUNT] = {
	[KS_SHIFT] = {"aA", 0, 17},
	[KS_NUM] = {"123", 18, 39},
	[KS_ACCENT] = {"\xc3\xa1\xc3\xa9", 40, 57},
	[KS_SPACE] = {"spc", 58, 87},
	[KS_DEL] = {"<-", 88, 105},
	[KS_DONE] = {"OK", 106, 127},
};

static struct {
	char title[22];
	char text[MESH_EV_TEXT_MAX + 1];
	size_t max_len;      /* in characters */
	enum kb_layout layout;
	bool one_shot;       /* shift: back to lowercase after one letter */
	int row, col;
	void (*done)(const char *text);
} editor;

static int utf8_chars(const char *s)
{
	int n = 0;

	while (*s) {
		gfx_utf8_next(&s);
		n++;
	}
	return n;
}

static void utf8_pop(char *s)
{
	size_t len = strlen(s);

	while (len > 0) {
		len--;
		if (((uint8_t)s[len] & 0xC0) != 0x80) { /* start of a character */
			break;
		}
	}
	s[len] = '\0';
}

static void open_editor(const char *title, const char *initial, size_t max_len,
			void (*done)(const char *text))
{
	snprintf(editor.title, sizeof(editor.title), "%s", title);
	snprintf(editor.text, sizeof(editor.text), "%s", initial ? initial : "");
	editor.max_len = max_len;
	editor.done = done;
	editor.layout = KB_LOWER;
	editor.one_shot = false;
	editor.row = 0;
	editor.col = 0;
	push((struct frame){.kind = K_EDIT_TEXT});
}

static void editor_type(const char *utf8)
{
	size_t len = strlen(editor.text);

	if ((size_t)utf8_chars(editor.text) >= editor.max_len ||
	    len + strlen(utf8) >= sizeof(editor.text)) {
		return;
	}
	strcat(editor.text, utf8);
	if (editor.one_shot) {
		editor.one_shot = false;
		editor.layout = KB_LOWER;
	}
}

static void editor_special(enum kb_special key)
{
	switch (key) {
	case KS_SHIFT:
		if (editor.layout == KB_LOWER) {
			editor.layout = KB_UPPER; /* one capital letter */
			editor.one_shot = true;
		} else if (editor.layout == KB_UPPER && editor.one_shot) {
			editor.one_shot = false;  /* second press: caps lock */
		} else {
			editor.layout = KB_LOWER;
			editor.one_shot = false;
		}
		break;
	case KS_NUM:
		editor.layout = editor.layout == KB_NUM ? KB_LOWER : KB_NUM;
		editor.one_shot = false;
		break;
	case KS_ACCENT:
		editor.layout = editor.layout == KB_ACCENT ? KB_LOWER : KB_ACCENT;
		editor.one_shot = false;
		break;
	case KS_SPACE:
		editor_type(" ");
		break;
	case KS_DEL:
		utf8_pop(editor.text);
		break;
	case KS_DONE:
		pop();
		if (editor.done) {
			editor.done(editor.text);
		}
		break;
	default:
		break;
	}
}

/* Text on two lines of 20 characters, showing the end and a cursor. */
static void draw_editor_text(void)
{
	char shown[3 * 40 + 2];
	char line[3 * 20 + 1];
	const char *p = editor.text;
	int n = utf8_chars(editor.text);
	int per_line = GFX_COLS - 1;

	for (int skip = MAX(0, ((n + 1 + per_line - 1) / per_line - 2) * per_line); skip > 0;
	     skip--) {
		gfx_utf8_next(&p);
	}
	snprintf(shown, sizeof(shown), "%s_", p);

	p = shown;
	for (int r = 0; r < 2 && *p; r++) {
		const char *start = p;

		for (int c = 0; c < per_line && *p; c++) {
			gfx_utf8_next(&p);
		}
		size_t len = MIN((size_t)(p - start), sizeof(line) - 1);

		memcpy(line, start, len);
		line[len] = '\0';
		gfx_text(1, KB_TEXT_ROW + r, line, GFX_W);
	}
}

static void draw_editor(void)
{
	char counter[12];
	static const char *const layout_names[KB_LAYOUTS] = {"abc", "ABC", "123", "\xc3\xa1\xc3\xa9"};

	snprintf(counter, sizeof(counter), "%s %d/%u",
		 editor.layout == KB_UPPER && !editor.one_shot ? "CAPS" : layout_names[editor.layout],
		 utf8_chars(editor.text), (unsigned)editor.max_len);
	ui_draw_header(editor.title, counter);
	draw_editor_text();

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < KB_COLS; c++) {
			int x = KB_X0 + c * KB_KEY_W;

			gfx_text(x + 3, KB_KEY_ROW + r, kb_keys[editor.layout][r][c], GFX_W);
			if (editor.row == r && editor.col == c) {
				gfx_invert_rect(x, (KB_KEY_ROW + r) * 8, x + KB_KEY_W - 2,
						(KB_KEY_ROW + r) * 8 + 7);
			}
		}
	}

	for (int k = 0; k < KS_COUNT; k++) {
		int x0 = kb_special_keys[k].x0, x1 = kb_special_keys[k].x1;
		int w = gfx_text_width(kb_special_keys[k].label);
		int y = (KB_KEY_ROW + 3) * 8;

		gfx_text(x0 + (x1 - x0 + 1 - w) / 2, KB_KEY_ROW + 3, kb_special_keys[k].label, GFX_W);
		if (editor.row == KB_SPECIAL && editor.col == k) {
			gfx_invert_rect(x0, y, x1 - 1, y + 7);
		} else {
			gfx_rect(x0, y, x1 - 1, y + 7);
		}
	}
	ui_draw_footer("OK type B2 row B0 del");
}

static void editor_input(const struct ui_input *in)
{
	int row_len = editor.row == KB_SPECIAL ? KS_COUNT : KB_COLS;

	if (in->rotate) {
		editor.col = ((editor.col + in->delta) % row_len + row_len) % row_len;
		return;
	}
	switch (in->key) {
	case UI_KEY_OK:
		if (editor.row == KB_SPECIAL) {
			editor_special((enum kb_special)editor.col);
		} else {
			editor_type(kb_keys[editor.layout][editor.row][editor.col]);
		}
		break;
	case UI_KEY_ACTION: {
		/* Next row, keeping the horizontal position roughly the same. */
		int next = (editor.row + 1) % KB_ROWS;

		if (next == KB_SPECIAL) {
			editor.col = editor.col * KS_COUNT / KB_COLS;
		} else if (editor.row == KB_SPECIAL) {
			editor.col = editor.col * KB_COLS / KS_COUNT;
		}
		editor.row = next;
		break;
	}
	case UI_KEY_BACK:
		if (editor.text[0] == '\0') {
			pop(); /* empty: leave without saving */
		} else {
			utf8_pop(editor.text);
		}
		break;
	case UI_KEY_HOME:
		go_home();
		break;
	}
}

/* ========================================================== number editor */

static struct {
	char title[22];
	char unit[6];
	int32_t value, min, max;
	int decimals;
	const int32_t *steps;
	int step_count;
	int step;
	void (*done)(int32_t value);
} numed;

static void open_number(const char *title, const char *unit, int32_t value, int32_t min,
			int32_t max, int decimals, const int32_t *steps, int step_count,
			int first_step, void (*done)(int32_t value))
{
	snprintf(numed.title, sizeof(numed.title), "%s", title);
	snprintf(numed.unit, sizeof(numed.unit), "%s", unit);
	numed.value = CLAMP(value, min, max);
	numed.min = min;
	numed.max = max;
	numed.decimals = decimals;
	numed.steps = steps;
	numed.step_count = step_count;
	numed.step = first_step;
	numed.done = done;
	push((struct frame){.kind = K_EDIT_NUM});
}

static void draw_number(void)
{
	char v[20], line[32], step[20];
	int w;

	ui_draw_header(numed.title, NULL);
	fmt_fixed(v, sizeof(v), numed.value, numed.decimals);
	snprintf(line, sizeof(line), "%s %s", v, numed.unit);
	w = gfx_text_width(line);
	gfx_text((GFX_W - w) / 2, 3, line, GFX_W);
	gfx_rect((GFX_W - w) / 2 - 4, 22, (GFX_W + w) / 2 + 2, 33);

	fmt_fixed(step, sizeof(step), numed.steps[numed.step], numed.decimals);
	snprintf(line, sizeof(line), "step: %s", step);
	ui_draw_line(4, line, 0);
	ui_draw_footer(numed.step_count > 1 ? "OK save  B2 step" : "OK save  B0 cancel");
}

static void number_input(const struct ui_input *in)
{
	if (in->rotate) {
		int64_t v = (int64_t)numed.value + (int64_t)in->delta * numed.steps[numed.step];

		numed.value = (int32_t)CLAMP(v, numed.min, numed.max);
		return;
	}
	switch (in->key) {
	case UI_KEY_OK:
		pop();
		numed.done(numed.value);
		break;
	case UI_KEY_ACTION:
		numed.step = (numed.step + 1) % numed.step_count;
		break;
	case UI_KEY_BACK:
		pop();
		break;
	case UI_KEY_HOME:
		go_home();
		break;
	}
}

/* ============================================================ confirmation */

static struct {
	char question[64];
	void (*yes)(void);
} confirm;

static void open_confirm(const char *question, void (*yes)(void))
{
	snprintf(confirm.question, sizeof(confirm.question), "%s", question);
	confirm.yes = yes;
	push((struct frame){.kind = K_CONFIRM});
}

/* ===================================================== lists: messages */

static int msgs_count(void)
{
	return history_count;
}

static void msgs_label(int i, char *buf, size_t size, void *ctx)
{
	struct msg_entry *m = history_get(i);
	char who[12];

	ARG_UNUSED(ctx);
	snprintf(buf, size, "%s%s%s:%s", m->read ? "" : "*", m->to != MESH_BROADCAST ? "@" : "",
		 node_name(m->from, who, sizeof(who)), m->text);
}

static const struct text_src msg_view;

static void msgs_select(int i)
{
	history_get(i)->read = true;
	push_text(&msg_view, (uint32_t)i);
}

static void canned_open(uint32_t to);

static void msgs_reply(int i)
{
	struct msg_entry *m = history_get(i);

	m->read = true;
	canned_open(m->to == MESH_BROADCAST ? MESH_BROADCAST : m->from);
}

static const struct list_src msgs_list = {
	.title = "Messages",
	.footer = "OK read  B2 reply",
	.count = msgs_count,
	.label = msgs_label,
	.select = msgs_select,
	.action = msgs_reply,
};

static void msg_build(char *buf, size_t size, uint32_t arg)
{
	struct msg_entry *m = history_get((int)arg);
	char who[12], ago[8];

	fmt_ago(ago, sizeof(ago), m->at_ms);
	snprintf(buf, size, "From %s, %s ago\n%s, %d dBm, SNR %d\n%s",
		 node_name(m->from, who, sizeof(who)), ago,
		 m->to == MESH_BROADCAST ? "channel" : "direct", m->rssi, m->snr, m->text);
}

static void msg_reply(uint32_t arg)
{
	msgs_reply((int)arg);
}

static const struct text_src msg_view = {
	.title = "Message",
	.footer = "B2 reply  B0 back",
	.build = msg_build,
	.action = msg_reply,
};

/* ======================================================== lists: nodes */

static int nodes_count(void)
{
	return node_count;
}

static void nodes_label(int i, char *buf, size_t size, void *ctx)
{
	const struct node_entry *n = &nodes[i];
	char ago[8];

	ARG_UNUSED(ctx);
	fmt_ago(ago, sizeof(ago), n->last_heard_ms);
	snprintf(buf, size, "%-4.4s %-9.9s %s", n->has_user ? n->short_name : "?",
		 n->has_user ? n->long_name : "", ago);
}

static const struct text_src node_view;

static void nodes_select(int i)
{
	push_text(&node_view, nodes[i].num);
}

static void nodes_dm(int i)
{
	canned_open(nodes[i].num);
}

static const struct list_src nodes_list = {
	.title = "Nodes",
	.footer = "OK info  B2 message",
	.count = nodes_count,
	.label = nodes_label,
	.select = nodes_select,
	.action = nodes_dm,
	.refresh = reload_nodes,
};

static void node_build(char *buf, size_t size, uint32_t arg)
{
	struct node_entry n;
	char ago[8], lat[16], lon[16];
	int len;

	if (!nodedb_get(arg, &n)) {
		snprintf(buf, size, "Node !%08x is no longer in the table.", arg);
		return;
	}
	fmt_ago(ago, sizeof(ago), n.last_heard_ms);
	len = snprintf(buf, size, "%s\n!%08x\n%s %s\nHeard %s ago, %u pkts\n",
		       n.has_user ? n.long_name : "(no name)", n.num,
		       n.has_user ? mesh_hw_model_name(n.hw_model) : "",
		       n.has_user ? mesh_role_name(n.role) : "", ago, n.packets);
	if (n.has_signal) {
		len += snprintf(buf + len, size - len, "Direct %d dBm SNR %d\n", n.rssi, n.snr);
	} else if (n.hops_away >= 0) {
		len += snprintf(buf + len, size - len, "%d hop(s)\n", n.hops_away);
	}
	if (n.has_battery) {
		len += snprintf(buf + len, size - len, "Battery %u%%\n", n.battery_level);
	}
	if (n.has_pos) {
		fmt_fixed(lat, sizeof(lat), n.lat_i / 100, 5);
		fmt_fixed(lon, sizeof(lon), n.lon_i / 100, 5);
		snprintf(buf + len, size - len, "%s, %s\n", lat, lon);
	}
}

static void node_dm(uint32_t arg)
{
	canned_open(arg);
}

static const struct text_src node_view = {
	.title = "Node",
	.footer = "B2 message  B0 back",
	.build = node_build,
	.action = node_dm,
};

/* ================================================ lists: send a message */

static const char *const canned[] = {
	"Oi!",
	"Tudo bem?",
	"OK, recebido",
	"Sim",
	"N\xc3\xa3o",
	"Estou a caminho",
	"Chego em 10 min",
	"Me liga",
	"Teste da ZBook",
};

static int canned_count(void)
{
	return ARRAY_SIZE(canned) + 1;
}

static void canned_label(int i, char *buf, size_t size, void *ctx)
{
	ARG_UNUSED(ctx);
	snprintf(buf, size, "%s", i == 0 ? "> Write..." : canned[i - 1]);
}

static void compose_done(const char *text)
{
	if (text[0] != '\0') {
		send_text_to(compose_to, text);
		go_home();
	}
}

static void canned_select(int i)
{
	if (i == 0) {
		open_editor(compose_to == MESH_BROADCAST ? "To all" : "Direct msg", "",
			    MESH_EV_TEXT_MAX, compose_done);
		return;
	}
	send_text_to(compose_to, canned[i - 1]);
	go_home();
}

static const struct list_src canned_list = {
	.title = "Send",
	.footer = "OK send  B0 back",
	.count = canned_count,
	.label = canned_label,
	.select = canned_select,
};

static void canned_open(uint32_t to)
{
	compose_to = to;
	push_list(&canned_list);
}

/* =========================================================== radio menu */

static int region_count(void)
{
	return (int)mesh_region_count;
}

static void region_label(int i, char *buf, size_t size, void *ctx)
{
	ARG_UNUSED(ctx);
	snprintf(buf, size, "%s %s", radio_get()->region == &mesh_regions[i] ? "*" : " ",
		 mesh_regions[i].name);
}

static void region_select(int i)
{
	radio_set(&mesh_regions[i], radio_get()->preset);
	pop();
	toast("Region changed");
}

static const struct list_src region_list = {
	.title = "Region",
	.footer = "OK select  B0 back",
	.count = region_count,
	.label = region_label,
	.select = region_select,
};

static int preset_count(void)
{
	return (int)mesh_preset_count;
}

static void preset_label(int i, char *buf, size_t size, void *ctx)
{
	ARG_UNUSED(ctx);
	snprintf(buf, size, "%s %s", radio_get()->preset == &mesh_presets[i] ? "*" : " ",
		 mesh_presets[i].name);
}

static void preset_select(int i)
{
	radio_set(radio_get()->region, &mesh_presets[i]);
	pop();
	toast("Preset changed");
}

static const struct list_src preset_list = {
	.title = "Preset",
	.footer = "OK select  B0 back",
	.count = preset_count,
	.label = preset_label,
	.select = preset_select,
};

static const int32_t steps_one[] = {1};

static void txpower_done(int32_t v)
{
	radio_set_tx_power((uint8_t)v);
	toast("TX power saved");
}

static void diag_build(char *buf, size_t size, uint32_t arg)
{
	struct radio_diag d, want;
	struct radio_stats rs;
	bool ok;

	ARG_UNUSED(arg);
	radio_get_stats(&rs);
	if (radio_read_diag(&d) < 0) {
		snprintf(buf, size, "SPI error reading the radio.");
		return;
	}
	radio_expected_diag(&want);
	ok = d.op_mode == want.op_mode && d.frf == want.frf &&
	     d.modem_config_1 == want.modem_config_1 &&
	     d.modem_config_2 == want.modem_config_2 &&
	     d.modem_config_3 == want.modem_config_3 && d.sync_word == want.sync_word &&
	     d.preamble == want.preamble;
	snprintf(buf, size,
		 "SX1276 v0x%02x\nRegisters: %s\nChannel RSSI: %d dBm\nDIO0: %s\n"
		 "RX %u  CRC err %u\nTX %u (%u ms on air)",
		 d.version, ok ? "OK" : "MISMATCH", d.rssi_now,
		 !rs.dio0_configured ? "no pin" : rs.dio0_hits ? "ok" : "no edge",
		 rs.rx_frames, rs.crc_errors, rs.tx_frames, rs.tx_airtime_ms);
}

static const struct text_src diag_view = {
	.title = "Diagnostics",
	.footer = "Turn to scroll  B0 <",
	.build = diag_build,
};

enum { RADIO_REGION, RADIO_PRESET, RADIO_POWER, RADIO_RELAY, RADIO_DIAG, RADIO_N };

static int radio_count(void)
{
	return RADIO_N;
}

static void radio_label(int i, char *buf, size_t size, void *ctx)
{
	const struct radio_state *s = radio_get();

	ARG_UNUSED(ctx);
	switch (i) {
	case RADIO_REGION: snprintf(buf, size, "Region: %s", s->region->name); break;
	case RADIO_PRESET: snprintf(buf, size, "Preset: %s", s->preset->name); break;
	case RADIO_POWER: snprintf(buf, size, "TX power: %u dBm", s->tx_power_dbm); break;
	case RADIO_RELAY:
		snprintf(buf, size, "Relay: %s", mesh_relay_enabled() ? "on" : "off");
		break;
	default: snprintf(buf, size, "Diagnostics"); break;
	}
}

static void radio_select(int i)
{
	switch (i) {
	case RADIO_REGION: push_list(&region_list); break;
	case RADIO_PRESET: push_list(&preset_list); break;
	case RADIO_POWER:
		open_number("TX power", "dBm", radio_get()->tx_power_dbm, 2, 17, 0, steps_one,
			    1, 0, txpower_done);
		break;
	case RADIO_RELAY: mesh_set_relay(!mesh_relay_enabled()); break;
	default: push_text(&diag_view, 0); break;
	}
}

static const struct list_src radio_list = {
	.title = "Radio",
	.footer = "OK change  B0 back",
	.count = radio_count,
	.label = radio_label,
	.select = radio_select,
};

/* ======================================================== position menu */

static const int32_t steps_deg[] = {1, 10, 100, 1000, 10000, 100000, 1000000}; /* 1e-5 deg */
static const int32_t steps_alt[] = {1, 10, 100};

static void lat_done(int32_t v)
{
	edit_lat = v;
}

static void lon_done(int32_t v)
{
	edit_lon = v;
}

static void alt_done(int32_t v)
{
	edit_alt = v;
}

enum { POS_LAT, POS_LON, POS_ALT, POS_SEND, POS_OFF, POS_N };

static int pos_count(void)
{
	return POS_N;
}

static void pos_label(int i, char *buf, size_t size, void *ctx)
{
	char v[16];

	ARG_UNUSED(ctx);
	switch (i) {
	case POS_LAT:
		fmt_fixed(v, sizeof(v), edit_lat, 5);
		snprintf(buf, size, "Lat: %s", v);
		break;
	case POS_LON:
		fmt_fixed(v, sizeof(v), edit_lon, 5);
		snprintf(buf, size, "Lon: %s", v);
		break;
	case POS_ALT: snprintf(buf, size, "Alt: %d m", edit_alt); break;
	case POS_SEND: snprintf(buf, size, "> Save and send"); break;
	default:
		snprintf(buf, size, "Disable (%s)", mesh_has_position() ? "active" : "inactive");
		break;
	}
}

static void pos_select(int i)
{
	switch (i) {
	case POS_LAT:
		open_number("Latitude", "", edit_lat, -9000000, 9000000, 5, steps_deg,
			    ARRAY_SIZE(steps_deg), 3, lat_done);
		break;
	case POS_LON:
		open_number("Longitude", "", edit_lon, -18000000, 18000000, 5, steps_deg,
			    ARRAY_SIZE(steps_deg), 3, lon_done);
		break;
	case POS_ALT:
		open_number("Altitude", "m", edit_alt, -500, 9000, 0, steps_alt,
			    ARRAY_SIZE(steps_alt), 0, alt_done);
		break;
	case POS_SEND:
		mesh_set_position(edit_lat * 100, edit_lon * 100, edit_alt);
		toast(mesh_send_position(MESH_BROADCAST) == 0 ? "Position sent" : "Failed");
		break;
	default:
		mesh_clear_position();
		toast("Position disabled");
		break;
	}
}

static void pos_refresh(void)
{
	int32_t lat, lon, alt;

	/* Start from the position in use, if there is one and nothing was edited. */
	if (edit_lat == 0 && edit_lon == 0 && mesh_get_position(&lat, &lon, &alt)) {
		edit_lat = lat / 100;
		edit_lon = lon / 100;
		edit_alt = alt;
	}
}

static const struct list_src pos_list = {
	.title = "Position",
	.footer = "OK edit  B0 back",
	.count = pos_count,
	.label = pos_label,
	.select = pos_select,
	.refresh = pos_refresh,
};

/* ===================================================== settings & name */

static void long_name_done(const char *text)
{
	if (text[0]) {
		mesh_set_name(text, NULL);
		mesh_send_nodeinfo(MESH_BROADCAST, false);
		toast("Name saved");
	}
}

static void short_name_done(const char *text)
{
	if (text[0]) {
		mesh_set_name(mesh_identity()->long_name, text);
		mesh_send_nodeinfo(MESH_BROADCAST, false);
		toast("Name saved");
	}
}

static void clear_nodes(void)
{
	nodedb_clear();
	reload_nodes();
	toast("Table cleared");
}

enum { SET_LONG, SET_SHORT, SET_SOUND, SET_SCREEN, SET_LOG, SET_CLEAR, SET_N };

static int set_count(void)
{
	return SET_N;
}

static void set_label(int i, char *buf, size_t size, void *ctx)
{
	int t = screen_timeouts_s[screen_timeout_idx];

	ARG_UNUSED(ctx);
	switch (i) {
	case SET_LONG: snprintf(buf, size, "Name: %s", mesh_identity()->long_name); break;
	case SET_SHORT: snprintf(buf, size, "Short: %s", mesh_identity()->short_name); break;
	case SET_SOUND: snprintf(buf, size, "Sound: %s", buzzer_enabled() ? "on" : "off"); break;
	case SET_SCREEN:
		if (t == 0) {
			snprintf(buf, size, "Screen off: never");
		} else {
			snprintf(buf, size, "Screen off: %d s", t);
		}
		break;
	case SET_LOG:
		snprintf(buf, size, "Serial log: %s", mesh_live_enabled() ? "on" : "off");
		break;
	default: snprintf(buf, size, "Clear node table"); break;
	}
}

static void set_select(int i)
{
	switch (i) {
	case SET_LONG:
		open_editor("Long name", mesh_identity()->long_name, 39, long_name_done);
		break;
	case SET_SHORT:
		open_editor("Short name", mesh_identity()->short_name, 4, short_name_done);
		break;
	case SET_SOUND: buzzer_set_enabled(!buzzer_enabled()); break;
	case SET_SCREEN:
		screen_timeout_idx = (screen_timeout_idx + 1) % ARRAY_SIZE(screen_timeouts_s);
		break;
	case SET_LOG: mesh_set_live(!mesh_live_enabled()); break;
	default: open_confirm("Delete all nodes from the table?", clear_nodes); break;
	}
}

static const struct list_src set_list = {
	.title = "Settings",
	.footer = "OK change  B0 back",
	.count = set_count,
	.label = set_label,
	.select = set_select,
};

/* ========================================================== about & help */

static void about_build(char *buf, size_t size, uint32_t arg)
{
	const struct mesh_identity *me = mesh_identity();
	const struct radio_state *s = radio_get();
	struct mesh_stats ms;
	int up = (int)(k_uptime_get() / 1000);

	ARG_UNUSED(arg);
	mesh_get_stats(&ms);
	snprintf(buf, size,
		 "%s (%s)\n!%08x\n%s %s %u.%03u MHz\nReceived %u\nSent %u\n"
		 "Relayed %u\nUp %dh%02dm",
		 me->long_name, me->short_name, me->num, s->region->name, s->preset->name,
		 s->freq_hz / 1000000U, (s->freq_hz / 1000U) % 1000U, ms.received, ms.sent,
		 ms.relayed, up / 3600, (up / 60) % 60);
}

static const struct text_src about_view = {
	.title = "About",
	.footer = "Turn to scroll  B0 <",
	.build = about_build,
};

static void help_build(char *buf, size_t size, uint32_t arg)
{
	ARG_UNUSED(arg);
	snprintf(buf, size,
		 "Turn: move/change\nPress or BTN1: OK\nBTN0: back\n"
		 "BTN2: action (footer)\nBTN3: home\nKeyboard: turn moves\nalong the row, BTN2\n"
		 "next row, OK types,\nBTN0 deletes, OK key\nfinishes.");
}

static const struct text_src help_view = {
	.title = "Help",
	.footer = "Turn to scroll  B0 <",
	.build = help_build,
};

/* ============================================================ main menu */

static void announce(void)
{
	toast(mesh_send_nodeinfo(MESH_BROADCAST, true) == 0 ? "Announced" : "Failed");
}

enum { MENU_MSGS, MENU_NODES, MENU_SEND, MENU_ANNOUNCE, MENU_RADIO, MENU_POS, MENU_SET,
       MENU_ABOUT, MENU_HELP, MENU_N };

static int menu_count(void)
{
	return MENU_N;
}

static void menu_label(int i, char *buf, size_t size, void *ctx)
{
	ARG_UNUSED(ctx);
	switch (i) {
	case MENU_MSGS: snprintf(buf, size, "Messages (%d)", history_count); break;
	case MENU_NODES: snprintf(buf, size, "Nodes (%d)", node_count); break;
	case MENU_SEND: snprintf(buf, size, "Send to all"); break;
	case MENU_ANNOUNCE: snprintf(buf, size, "Announce this node"); break;
	case MENU_RADIO: snprintf(buf, size, "Radio"); break;
	case MENU_POS: snprintf(buf, size, "Position"); break;
	case MENU_SET: snprintf(buf, size, "Settings"); break;
	case MENU_ABOUT: snprintf(buf, size, "About"); break;
	default: snprintf(buf, size, "Button help"); break;
	}
}

static void menu_select(int i)
{
	switch (i) {
	case MENU_MSGS: push_list(&msgs_list); break;
	case MENU_NODES: push_list(&nodes_list); break;
	case MENU_SEND: canned_open(MESH_BROADCAST); break;
	case MENU_ANNOUNCE: announce(); break;
	case MENU_RADIO: push_list(&radio_list); break;
	case MENU_POS: push_list(&pos_list); break;
	case MENU_SET: push_list(&set_list); break;
	case MENU_ABOUT: push_text(&about_view, 0); break;
	default: push_text(&help_view, 0); break;
	}
}

static const struct list_src main_menu = {
	.title = "Menu",
	.footer = "OK open  B0 back",
	.count = menu_count,
	.label = menu_label,
	.select = menu_select,
	.refresh = reload_nodes,
};

/* ============================================================ rendering */

static void draw_home(void)
{
	const struct mesh_identity *me = mesh_identity();
	const struct radio_state *s = radio_get();
	struct mesh_stats ms;
	char line[48], who[12];
	int unread = unread_count();

	mesh_get_stats(&ms);
	snprintf(line, sizeof(line), unread ? "%d new" : "", unread);
	ui_draw_header(me->long_name, unread ? line : NULL);

	snprintf(line, sizeof(line), "!%08x  %s", me->num, me->short_name);
	ui_draw_line(0, line, 0);
	snprintf(line, sizeof(line), "%s %s", s->region->name, s->preset->name);
	ui_draw_line(1, line, 0);
	snprintf(line, sizeof(line), "%u.%03u MHz  %u dBm", s->freq_hz / 1000000U,
		 (s->freq_hz / 1000U) % 1000U, s->tx_power_dbm);
	ui_draw_line(2, line, 0);
	snprintf(line, sizeof(line), "Nodes %d  RX %u  TX %u", node_count, ms.received, ms.sent);
	ui_draw_line(3, line, 0);

	if (history_count > 0) {
		struct msg_entry *m = history_get(0);

		snprintf(line, sizeof(line), "%s: %s", node_name(m->from, who, sizeof(who)), m->text);
		ui_draw_line(5, line, 0);
	} else {
		ui_draw_line(5, "No messages", 0);
	}
	ui_draw_footer("OK menu  B2 announce");
}

static void draw_list(struct frame *f)
{
	char right[8];
	int count;

	if (f->list->refresh) {
		f->list->refresh();
	}
	count = f->list->count();
	if (f->sel >= count) {
		f->sel = count > 0 ? count - 1 : 0;
	}
	snprintf(right, sizeof(right), "%d/%d", count ? f->sel + 1 : 0, count);
	ui_draw_header(f->list->title, right);
	ui_draw_list(count, f->sel, &f->top, f->list->label, NULL);
	ui_draw_footer(f->list->footer);
}

static void draw_text(struct frame *f)
{
	static char body[512];
	int total;

	f->text->build(body, sizeof(body), f->arg);
	ui_draw_header(f->text->title, NULL);
	total = ui_draw_wrapped(body, 0, UI_CONTENT_ROWS, f->scroll);
	if (f->scroll > 0 && f->scroll > total - UI_CONTENT_ROWS) {
		f->scroll = MAX(0, total - UI_CONTENT_ROWS);
	}
	ui_draw_footer(f->text->footer);
}

static void draw_confirm(void)
{
	ui_draw_header("Confirm", NULL);
	ui_draw_wrapped(confirm.question, 1, 4, 0);
	ui_draw_footer("OK yes  B0 no");
}

static void render(void)
{
	struct frame *f = cur();

	gfx_clear();
	switch (f->kind) {
	case K_HOME: draw_home(); break;
	case K_LIST: draw_list(f); break;
	case K_TEXT: draw_text(f); break;
	case K_EDIT_TEXT: draw_editor(); break;
	case K_EDIT_NUM: draw_number(); break;
	case K_CONFIRM: draw_confirm(); break;
	}
	if (k_uptime_get() < popup_until) {
		ui_draw_popup(popup_title, popup_body);
	}
}

/* ============================================================== display */

static const struct device *const disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static bool invert_pixels;
static bool blanked;

static bool display_init(void)
{
	struct display_capabilities caps;

	if (!device_is_ready(disp)) {
		LOG_ERR("OLED not ready: UI disabled");
		return false;
	}
	display_set_pixel_format(disp, PIXEL_FORMAT_MONO01);
	display_get_capabilities(disp, &caps);
	if (caps.x_resolution != GFX_W || caps.y_resolution != GFX_H ||
	    !(caps.screen_info & SCREEN_INFO_MONO_VTILED)) {
		LOG_ERR("Unexpected display (%ux%u): UI disabled", caps.x_resolution,
			caps.y_resolution);
		return false;
	}
	/* Our framebuffer uses 1 = lit pixel. */
	invert_pixels = caps.current_pixel_format == PIXEL_FORMAT_MONO10;
	display_blanking_off(disp);
	return true;
}

static void flush(void)
{
	static uint8_t out[GFX_ROWS * GFX_W];
	const uint8_t *fb = gfx_buffer();
	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(out),
		.width = GFX_W,
		.height = GFX_H,
		.pitch = GFX_W,
	};

	for (size_t i = 0; i < sizeof(out); i++) {
		out[i] = invert_pixels ? (uint8_t)~fb[i] : fb[i];
	}
	display_write(disp, 0, 0, &desc, out);
}

static void set_blank(bool on)
{
	if (on != blanked) {
		blanked = on;
		if (on) {
			display_blanking_on(disp);
		} else {
			display_blanking_off(disp);
		}
	}
}

/* ================================================================ input */

static void handle_input(const struct ui_input *in)
{
	struct frame *f = cur();

	switch (f->kind) {
	case K_EDIT_TEXT:
		editor_input(in);
		return;
	case K_EDIT_NUM:
		number_input(in);
		return;
	default:
		break;
	}

	if (in->rotate) {
		if (f->kind == K_LIST) {
			int count = f->list->count();

			if (count > 0) {
				f->sel = CLAMP(f->sel + in->delta, 0, count - 1);
			}
		} else if (f->kind == K_TEXT) {
			f->scroll = MAX(0, f->scroll + in->delta);
		}
		return;
	}

	switch (in->key) {
	case UI_KEY_HOME:
		go_home();
		break;
	case UI_KEY_BACK:
		pop();
		break;
	case UI_KEY_OK:
		if (f->kind == K_HOME) {
			push_list(&main_menu);
		} else if (f->kind == K_LIST && f->list->count() > 0) {
			f->list->select(f->sel);
		} else if (f->kind == K_CONFIRM) {
			void (*yes)(void) = confirm.yes;

			pop();
			yes();
		}
		break;
	case UI_KEY_ACTION:
		if (f->kind == K_HOME) {
			announce();
		} else if (f->kind == K_LIST && f->list->action && f->list->count() > 0) {
			f->list->action(f->sel);
		} else if (f->kind == K_TEXT && f->text->action) {
			f->text->action(f->arg);
		}
		break;
	}
}

/* ========================================================== mesh events */

static void handle_mesh_event(const struct mesh_event *ev)
{
	char who[12], title[24];

	switch (ev->type) {
	case MESH_EV_TEXT: {
		struct msg_entry *m = &history[history_head];

		m->from = ev->from;
		m->to = ev->to;
		m->rssi = ev->rssi;
		m->snr = ev->snr;
		m->read = false;
		m->at_ms = k_uptime_get();
		snprintf(m->text, sizeof(m->text), "%s", ev->text);
		history_head = (history_head + 1) % HISTORY_LEN;
		history_count = MIN(history_count + 1, HISTORY_LEN);

		snprintf(title, sizeof(title), "%s %s", ev->to == MESH_BROADCAST ? "Msg from" : "DM from",
			 node_name(ev->from, who, sizeof(who)));
		popup(title, ev->text, POPUP_MS);
		break;
	}
	case MESH_EV_NODE:
		reload_nodes();
		break;
	case MESH_EV_ACK:
		if (ev->id == pending_dm_id) {
			snprintf(title, sizeof(title), "%s %s", ev->ok ? "Delivered to" : "Failed to",
				 node_name(ev->from, who, sizeof(who)));
			popup(title, "", TOAST_MS * 2);
			pending_dm_id = 0;
		}
		break;
	default:
		break;
	}
}

/* =============================================================== thread */

ZBUS_MSG_SUBSCRIBER_DEFINE(ui_sub);
ZBUS_CHAN_ADD_OBS(ui_input_chan, ui_sub, 3);
ZBUS_CHAN_ADD_OBS(mesh_event_chan, ui_sub, 3);

static K_SEM_DEFINE(ui_started, 0, 1);

static void ui_thread(void *a, void *b, void *c)
{
	const struct zbus_channel *chan;
	union {
		struct ui_input in;
		struct mesh_event ev;
	} msg;
	int64_t last_activity = 0, last_draw = 0;
	bool ready;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	k_sem_take(&ui_started, K_FOREVER);
	ready = display_init();

	stack[0] = (struct frame){.kind = K_HOME};
	depth = 1;
	reload_nodes();
	last_activity = k_uptime_get();

	while (1) {
		bool dirty = false;
		int64_t now;

		if (zbus_sub_wait_msg(&ui_sub, &chan, &msg, K_MSEC(TICK_MS)) == 0) {
			if (chan == &ui_input_chan) {
				last_activity = k_uptime_get();
				if (blanked) {
					set_blank(false); /* the first touch only wakes the screen */
				} else {
					popup_until = 0;
					handle_input(&msg.in);
				}
			} else if (chan == &mesh_event_chan) {
				handle_mesh_event(&msg.ev);
				if (msg.ev.type == MESH_EV_TEXT) {
					last_activity = k_uptime_get();
					set_blank(false);
				}
			}
			dirty = true;
		}

		now = k_uptime_get();
		if (screen_timeouts_s[screen_timeout_idx] > 0 &&
		    now - last_activity > screen_timeouts_s[screen_timeout_idx] * 1000LL) {
			set_blank(true);
		}
		/* Refresh about once a second for clocks, counters and popups. */
		if (now - last_draw >= 1000) {
			dirty = true;
		}

		if (ready && dirty && !blanked) {
			render();
			flush();
			last_draw = now;
		}
	}
}

K_THREAD_DEFINE(ui_tid, 4096, ui_thread, NULL, NULL, NULL, 10, 0, 0);

void ui_start(void)
{
	k_sem_give(&ui_started);
}
