/**
 * @file ui_draw.c
 * @brief Screen building blocks (header, footer, lists, wrapped text, popup).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "gfx.h"
#include "ui_draw.h"

void ui_draw_header(const char *title, const char *right)
{
	int right_w = right ? gfx_text_width(right) : 0;

	gfx_text(1, 0, title, GFX_W - right_w - 2);
	if (right) {
		gfx_text(GFX_W - right_w, 0, right, GFX_W);
	}
	gfx_invert_row(0);
}

void ui_draw_footer(const char *hint)
{
	gfx_hline(0, GFX_W - 1, 7 * 8);
	gfx_text(0, 7, hint, GFX_W);
}

void ui_draw_line(int line, const char *text, int inverted)
{
	int row = UI_FIRST_ROW + line;

	gfx_text(1, row, text, GFX_W);
	if (inverted) {
		gfx_invert_row(row);
	}
}

void ui_draw_list(int count, int sel, int *top, ui_label_fn label, void *ctx)
{
	char buf[64];

	if (count <= 0) {
		ui_draw_line(1, "  (empty)", 0);
		return;
	}
	if (sel < *top) {
		*top = sel;
	}
	if (sel >= *top + UI_CONTENT_ROWS) {
		*top = sel - UI_CONTENT_ROWS + 1;
	}

	for (int line = 0; line < UI_CONTENT_ROWS && *top + line < count; line++) {
		int i = *top + line;

		label(i, buf, sizeof(buf), ctx);
		ui_draw_line(line, buf, i == sel);
	}

	/* Scrollbar on the right edge when the list does not fit. */
	if (count > UI_CONTENT_ROWS) {
		int y0 = UI_FIRST_ROW * 8;
		int h = UI_CONTENT_ROWS * 8;
		int bar = h * UI_CONTENT_ROWS / count;
		int pos = (h - bar) * *top / (count - UI_CONTENT_ROWS);

		if (bar < 3) {
			bar = 3;
		}
		gfx_fill(GFX_W - 2, y0, GFX_W - 1, y0 + h - 1, false);
		gfx_vline(GFX_W - 1, y0 + pos, y0 + pos + bar - 1);
	}
}

/* Copy up to `cols` characters of one wrapped line into out; advance *s. */
static int next_line(const char **s, char *out, size_t size, int cols)
{
	const char *p = *s;
	const char *line_start = p;
	const char *last_space = NULL;
	int n = 0;

	if (*p == '\0') {
		return 0;
	}

	while (*p && *p != '\n' && n < cols) {
		const char *before = p;

		if (*p == ' ') {
			last_space = before;
		}
		gfx_utf8_next(&p);
		n++;
	}

	const char *end = p;

	if (*p && *p != '\n' && *p != ' ' && last_space) {
		end = last_space; /* break at the last space that fits */
		p = last_space;
	}

	size_t len = (size_t)(end - line_start);

	if (len >= size) {
		len = size - 1;
	}
	memcpy(out, line_start, len);
	out[len] = '\0';

	while (*p == ' ') {
		p++;
	}
	if (*p == '\n') {
		p++;
	}
	*s = p;
	return 1;
}

int ui_draw_wrapped(const char *utf8, int first, int max_lines, int skip)
{
	char line[3 * GFX_COLS + 1];
	int total = 0;

	while (next_line(&utf8, line, sizeof(line), GFX_COLS)) {
		int shown = total - skip;

		if (shown >= 0 && shown < max_lines) {
			ui_draw_line(first + shown, line, 0);
		}
		total++;
	}
	return total;
}

void ui_draw_popup(const char *title, const char *utf8)
{
	char line[3 * GFX_COLS + 1];
	int row = 2;

	/* Box over the whole content area: title on row 1, text on rows 2..5. */
	gfx_fill(0, 8, GFX_W - 1, 55, false);
	gfx_rect(1, 8, GFX_W - 2, 55);
	gfx_text(5, 1, title, GFX_W - 4);
	gfx_hline(2, GFX_W - 3, 16);

	while (row <= 5 && next_line(&utf8, line, sizeof(line), GFX_COLS - 2)) {
		gfx_text(5, row, line, GFX_W - 4);
		row++;
	}
}
