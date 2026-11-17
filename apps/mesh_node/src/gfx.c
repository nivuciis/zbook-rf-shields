/**
 * @file gfx.c
 * @brief Minimal 1-bpp framebuffer for the 128x64 SSD1306 (vertical tiles).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "font5x8.h"
#include "gfx.h"

#define MAX_I(a, b) ((a) > (b) ? (a) : (b))

static uint8_t fb[GFX_ROWS * GFX_W];

void gfx_clear(void)
{
	memset(fb, 0, sizeof(fb));
}

const uint8_t *gfx_buffer(void)
{
	return fb;
}

void gfx_pixel(int x, int y, bool on)
{
	if (x < 0 || x >= GFX_W || y < 0 || y >= GFX_H) {
		return;
	}
	uint8_t *b = &fb[(y / 8) * GFX_W + x];

	if (on) {
		*b |= (uint8_t)(1u << (y % 8));
	} else {
		*b &= (uint8_t)~(1u << (y % 8));
	}
}

void gfx_hline(int x0, int x1, int y)
{
	for (int x = x0; x <= x1; x++) {
		gfx_pixel(x, y, true);
	}
}

void gfx_vline(int x, int y0, int y1)
{
	for (int y = y0; y <= y1; y++) {
		gfx_pixel(x, y, true);
	}
}

void gfx_rect(int x0, int y0, int x1, int y1)
{
	gfx_hline(x0, x1, y0);
	gfx_hline(x0, x1, y1);
	gfx_vline(x0, y0, y1);
	gfx_vline(x1, y0, y1);
}

void gfx_fill(int x0, int y0, int x1, int y1, bool on)
{
	for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			gfx_pixel(x, y, on);
		}
	}
}

void gfx_invert_row(int row)
{
	if (row < 0 || row >= GFX_ROWS) {
		return;
	}
	for (int x = 0; x < GFX_W; x++) {
		fb[row * GFX_W + x] ^= 0xFF;
	}
}

void gfx_invert_rect(int x0, int y0, int x1, int y1)
{
	for (int y = MAX_I(y0, 0); y <= y1 && y < GFX_H; y++) {
		for (int x = MAX_I(x0, 0); x <= x1 && x < GFX_W; x++) {
			fb[(y / 8) * GFX_W + x] ^= (uint8_t)(1u << (y % 8));
		}
	}
}

uint32_t gfx_utf8_next(const char **s)
{
	const uint8_t *p = (const uint8_t *)*s;
	uint32_t cp;
	int extra;

	if (p[0] < 0x80) {
		cp = p[0];
		extra = 0;
	} else if ((p[0] & 0xE0) == 0xC0) {
		cp = p[0] & 0x1F;
		extra = 1;
	} else if ((p[0] & 0xF0) == 0xE0) {
		cp = p[0] & 0x0F;
		extra = 2;
	} else if ((p[0] & 0xF8) == 0xF0) {
		cp = p[0] & 0x07;
		extra = 3;
	} else {
		*s += 1;
		return '?';
	}

	for (int i = 1; i <= extra; i++) {
		if ((p[i] & 0xC0) != 0x80) {
			*s += i;
			return '?';
		}
		cp = (cp << 6) | (p[i] & 0x3F);
	}
	*s += 1 + extra;
	return cp;
}

size_t gfx_utf8_put(char *out, size_t size, uint32_t cp)
{
	if (cp < 0x80 && size >= 1) {
		out[0] = (char)cp;
		return 1;
	}
	if (cp < 0x800 && size >= 2) {
		out[0] = (char)(0xC0 | (cp >> 6));
		out[1] = (char)(0x80 | (cp & 0x3F));
		return 2;
	}
	return 0;
}

static const uint8_t *glyph(uint32_t cp)
{
	if (cp >= 0x20 && cp <= 0x7E) {
		return font5x8_ascii[cp - 0x20];
	}
	if (cp >= 0xA0 && cp <= 0xFF) {
		return font5x8_latin1[cp - 0xA0];
	}
	return font5x8_ascii['?' - 0x20];
}

int gfx_text(int x, int row, const char *utf8, int x_max)
{
	if (row < 0 || row >= GFX_ROWS) {
		return x;
	}
	if (x_max > GFX_W) {
		x_max = GFX_W;
	}

	while (*utf8 && x + FONT_W <= x_max) {
		const uint8_t *g = glyph(gfx_utf8_next(&utf8));

		for (int c = 0; c < FONT_W; c++) {
			if (x + c >= 0) {
				fb[row * GFX_W + x + c] |= g[c];
			}
		}
		x += GFX_CELL_W;
	}
	return x;
}

int gfx_text_width(const char *utf8)
{
	int n = 0;

	while (*utf8) {
		gfx_utf8_next(&utf8);
		n++;
	}
	return n * GFX_CELL_W;
}
