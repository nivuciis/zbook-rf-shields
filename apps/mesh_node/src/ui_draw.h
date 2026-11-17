/**
 * @file ui_draw.h
 * @brief Screen building blocks (header, footer, lists, wrapped text, popup).
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 *
 * Layout of the 128x64 screen in text rows (8 px each):
 *   row 0      header (inverted)
 *   rows 1..6  content (UI_CONTENT_ROWS lines)
 *   row 7      footer with the button hints
 * Pure C on top of gfx.c, no Zephyr dependency.
 */
#ifndef UI_DRAW_H_
#define UI_DRAW_H_

#include <stddef.h>

#define UI_FIRST_ROW    1
#define UI_CONTENT_ROWS 6

/** Fills buf with the label of list item i. */
typedef void (*ui_label_fn)(int i, char *buf, size_t size, void *ctx);

void ui_draw_header(const char *title, const char *right);
void ui_draw_footer(const char *hint);

/**
 * Draw a scrolling list on the content rows. *top is the first visible item
 * and is adjusted so that sel stays visible. The selected item is inverted.
 */
void ui_draw_list(int count, int sel, int *top, ui_label_fn label, void *ctx);

/** Draw one content line (0..UI_CONTENT_ROWS-1), optionally inverted. */
void ui_draw_line(int line, const char *text, int inverted);

/**
 * Word-wrap text on content lines first..first+max_lines-1, skipping the
 * first `skip` wrapped lines (for scrolling).
 * @return total number of wrapped lines.
 */
int ui_draw_wrapped(const char *utf8, int first, int max_lines, int skip);

/** Centered box over the content area with a title and wrapped text. */
void ui_draw_popup(const char *title, const char *utf8);

#endif /* UI_DRAW_H_ */
