/**
 * @file pb.c
 * @brief Tiny protobuf wire-format reader and writer.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "pb.h"

void pb_reader_init(struct pb_reader *r, const uint8_t *buf, size_t len)
{
	r->p = buf;
	r->end = buf + len;
	r->error = false;
}

static bool read_varint(struct pb_reader *r, uint64_t *out)
{
	uint64_t v = 0;

	for (int shift = 0; shift < 64; shift += 7) {
		if (r->p >= r->end) {
			return false;
		}
		uint8_t b = *r->p++;

		v |= (uint64_t)(b & 0x7f) << shift;
		if (!(b & 0x80)) {
			*out = v;
			return true;
		}
	}
	return false;
}

static bool read_fixed(struct pb_reader *r, size_t n, uint64_t *out)
{
	uint64_t v = 0;

	if ((size_t)(r->end - r->p) < n) {
		return false;
	}
	for (size_t i = 0; i < n; i++) {
		v |= (uint64_t)r->p[i] << (8 * i); /* little-endian */
	}
	r->p += n;
	*out = v;
	return true;
}

bool pb_next(struct pb_reader *r, struct pb_field *f)
{
	uint64_t key, len;

	if (r->error || r->p >= r->end) {
		return false;
	}
	if (!read_varint(r, &key) || (key >> 3) == 0) {
		goto fail;
	}

	f->num = (uint32_t)(key >> 3);
	f->wire = (enum pb_wire)(key & 0x7);
	f->value = 0;
	f->data = NULL;
	f->len = 0;

	switch (f->wire) {
	case PB_VARINT:
		if (!read_varint(r, &f->value)) {
			goto fail;
		}
		break;
	case PB_FIXED64:
		if (!read_fixed(r, 8, &f->value)) {
			goto fail;
		}
		break;
	case PB_FIXED32:
		if (!read_fixed(r, 4, &f->value)) {
			goto fail;
		}
		break;
	case PB_LEN:
		if (!read_varint(r, &len) || len > (uint64_t)(r->end - r->p)) {
			goto fail;
		}
		f->data = r->p;
		f->len = (size_t)len;
		r->p += len;
		break;
	default:
		goto fail; /* groups (3, 4) and invalid types */
	}
	return true;

fail:
	r->error = true;
	return false;
}

void pb_copy_string(const struct pb_field *f, char *dst, size_t dst_size)
{
	size_t n = 0;

	if (dst_size == 0) {
		return;
	}
	if (f->wire == PB_LEN) {
		n = f->len < dst_size - 1 ? f->len : dst_size - 1;
		memcpy(dst, f->data, n);
	}
	dst[n] = '\0';
}

/* ------------------------------------------------------------- writer */

void pb_writer_init(struct pb_writer *w, uint8_t *buf, size_t size)
{
	w->buf = buf;
	w->size = size;
	w->len = 0;
	w->overflow = false;
}

static void put_byte(struct pb_writer *w, uint8_t b)
{
	if (w->len < w->size) {
		w->buf[w->len++] = b;
	} else {
		w->overflow = true;
	}
}

static void put_varint(struct pb_writer *w, uint64_t v)
{
	do {
		uint8_t b = v & 0x7f;

		v >>= 7;
		put_byte(w, v ? (b | 0x80) : b);
	} while (v);
}

void pb_write_varint(struct pb_writer *w, uint32_t field, uint64_t value)
{
	put_varint(w, ((uint64_t)field << 3) | PB_VARINT);
	put_varint(w, value);
}

void pb_write_fixed32(struct pb_writer *w, uint32_t field, uint32_t value)
{
	put_varint(w, ((uint64_t)field << 3) | PB_FIXED32);
	for (int i = 0; i < 4; i++) {
		put_byte(w, (uint8_t)(value >> (8 * i)));
	}
}

void pb_write_bytes(struct pb_writer *w, uint32_t field, const void *data, size_t len)
{
	const uint8_t *p = data;

	put_varint(w, ((uint64_t)field << 3) | PB_LEN);
	put_varint(w, len);
	for (size_t i = 0; i < len; i++) {
		put_byte(w, p[i]);
	}
}

void pb_write_string(struct pb_writer *w, uint32_t field, const char *str)
{
	pb_write_bytes(w, field, str, strlen(str));
}
