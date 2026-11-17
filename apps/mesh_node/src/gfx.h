/**
 * @file gfx.h
 * @brief Minimal 1-bpp framebuffer for the 128x64 SSD1306 (vertical tiles).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure C, no Zephyr dependency, so it can be unit-tested on a PC.
 * Text is drawn on 8-pixel "rows" (the SSD1306 pages): 8 rows of 21
 * characters with the 6-pixel cell of the 5x8 font.
 */
#ifndef GFX_H_
#define GFX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GFX_W      128
#define GFX_H      64
#define GFX_ROWS   (GFX_H / 8)
#define GFX_CELL_W 6
#define GFX_COLS   (GFX_W / GFX_CELL_W)  /* 21 */

void gfx_clear(void);

/** Framebuffer: byte (row * GFX_W + x), bit n = pixel (x, row * 8 + n). */
const uint8_t *gfx_buffer(void);

void gfx_pixel(int x, int y, bool on);
void gfx_hline(int x0, int x1, int y);
void gfx_vline(int x, int y0, int y1);
void gfx_rect(int x0, int y0, int x1, int y1);       /* outline */
void gfx_fill(int x0, int y0, int x1, int y1, bool on);
void gfx_invert_row(int row);                         /* whole text row */
void gfx_invert_rect(int x0, int y0, int x1, int y1); /* XOR a rectangle */

/**
 * Draw UTF-8 text on a text row, starting at pixel x and clipped at x_max.
 * Characters outside ASCII/Latin-1 are drawn as '?'.
 * @return the x position after the last character drawn.
 */
int gfx_text(int x, int row, const char *utf8, int x_max);

/** Width in pixels of a UTF-8 string. */
int gfx_text_width(const char *utf8);

/** Decode one UTF-8 character, return its code point and advance *s. */
uint32_t gfx_utf8_next(const char **s);

/** Append a code point (U+0000..U+00FF) to a UTF-8 buffer. Returns bytes written. */
size_t gfx_utf8_put(char *out, size_t size, uint32_t cp);

#endif /* GFX_H_ */
