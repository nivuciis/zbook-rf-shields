/**
 * @file pb.h
 * @brief Tiny protobuf wire-format reader and writer.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PB_H_
#define PB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum pb_wire {
	PB_VARINT = 0,
	PB_FIXED64 = 1,
	PB_LEN = 2,
	PB_FIXED32 = 5,
};

/** One decoded field. For PB_LEN, data/len point into the input buffer. */
struct pb_field {
	uint32_t num;
	enum pb_wire wire;
	uint64_t value;      /* PB_VARINT, PB_FIXED32, PB_FIXED64 */
	const uint8_t *data; /* PB_LEN */
	size_t len;          /* PB_LEN */
};

struct pb_reader {
	const uint8_t *p;
	const uint8_t *end;
	bool error;
};

void pb_reader_init(struct pb_reader *r, const uint8_t *buf, size_t len);

/**
 * Read the next field.
 * @return true if a field was read; false at end of buffer or on malformed
 *         input (check r->error to tell the two apart).
 */
bool pb_next(struct pb_reader *r, struct pb_field *f);

/** Copy a PB_LEN field into a NUL-terminated string, truncating if needed. */
void pb_copy_string(const struct pb_field *f, char *dst, size_t dst_size);

/** Append-only protobuf writer into a fixed buffer. */
struct pb_writer {
	uint8_t *buf;
	size_t size;
	size_t len;
	bool overflow;
};

void pb_writer_init(struct pb_writer *w, uint8_t *buf, size_t size);
void pb_write_varint(struct pb_writer *w, uint32_t field, uint64_t value);
void pb_write_fixed32(struct pb_writer *w, uint32_t field, uint32_t value);
void pb_write_bytes(struct pb_writer *w, uint32_t field, const void *data, size_t len);
void pb_write_string(struct pb_writer *w, uint32_t field, const char *str);

#endif /* PB_H_ */
