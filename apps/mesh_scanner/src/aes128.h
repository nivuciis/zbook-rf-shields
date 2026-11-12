/**
 * @file aes128.h
 * @brief Minimal AES-128 (encrypt direction only) and AES-128-CTR.
 * @copyright Copyright (c) 2026 Centro de Inovacao EDGE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef AES128_H_
#define AES128_H_

#include <stddef.h>
#include <stdint.h>

/** Expanded key: 11 round keys of 16 bytes. */
struct aes128_ctx {
	uint8_t round_key[176];
};

void aes128_init(struct aes128_ctx *ctx, const uint8_t key[16]);
void aes128_encrypt_block(const struct aes128_ctx *ctx, const uint8_t in[16], uint8_t out[16]);

/**
 * AES-128-CTR, in place. Encryption and decryption are the same operation.
 * The last 4 bytes of the 16-byte IV are a big-endian block counter, as in
 * the Meshtastic firmware (rweather Crypto, counter size 4).
 */
void aes128_ctr_xcrypt(const struct aes128_ctx *ctx, const uint8_t iv[16], uint8_t *buf, size_t len);

#endif /* AES128_H_ */
