/*
 * pukcc.h
 *
 * Public API for the PUKCC hardware ECC accelerator on SAME5x.
 * Provides P-256 scalar multiplication, ECDSA sign and verify.
 * All buffers are 32-byte big-endian.
 */

#ifndef PUKCC_H_INCLUDED
#define PUKCC_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enable PUKCC clock and run self-test.  Returns 0 on success. */
int ecc_p256_init(void);

/* R = k * P on secp256r1.  Pass px/py = NULL to use the generator G. */
int ecc_p256_mul(
    const uint8_t *scalar,
    const uint8_t *px, const uint8_t *py,
    uint8_t *rx, uint8_t *ry);

/* ECDSA sign: (sig_r, sig_s) = Sign(hash, private_key, nonce_k). */
int ecc_p256_ecdsa_sign(
    const uint8_t *hash,
    const uint8_t *private_key,
    const uint8_t *nonce_k,
    uint8_t *sig_r, uint8_t *sig_s);

/* ECDSA verify: returns 0 if signature (sig_r, sig_s) is valid. */
int ecc_p256_ecdsa_verify(
    const uint8_t *hash,
    const uint8_t *pub_x, const uint8_t *pub_y,
    const uint8_t *sig_r, const uint8_t *sig_s);

#ifdef __cplusplus
}
#endif

#endif /* PUKCC_H_INCLUDED */
