/**
 * @file font5x8.h
 * @brief 5x8 bitmap font (public-domain X11 misc-fixed), column-major glyphs.
 */
#ifndef FONT5X8_H_
#define FONT5X8_H_

#include <stdint.h>

#define FONT_W 5
#define FONT_H 8

extern const uint8_t font5x8_ascii[95][5];   /* U+0020..U+007E */
extern const uint8_t font5x8_latin1[96][5];  /* U+00A0..U+00FF */

#endif /* FONT5X8_H_ */
