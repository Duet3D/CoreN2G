/*
 * pukcc.c
 *
 * Hardware P-256 ECC operations using the PUKCC coprocessor
 * on SAME5x (SAME54/SAME51).  Uses the PUKCL ROM services:
 *   - ZpEccMulFast + ZpEcConvProjToAffine for scalar multiplication
 *   - ZpEcDsaGenerateFast for ECDSA signing
 *   - ZpEcDsaVerifyFast for ECDSA verification
 *
 * Thread safety: only called from the networking task (no locking needed).
 */

#if defined(__SAME54P20A__) || defined(__SAME51N19A__)

#include "pukcc.h"

#include <string.h>
#include <stdint.h>
#include <ecv_duet3d.h>
#if defined(__SAME54P20A__) || defined(__SAME51P20A__)
# include <same54.h>
#elif defined(__SAME51N19A__)
# include <same51.h>
#endif

#include "CryptoLib_Headers_pb.h"
#include "CryptoLib_Hardware_pb.h"


/* ------------------------------------------------------------------ */
/* PUKCL parameter block (shared with ROM)                            */
/* ------------------------------------------------------------------ */
static PUKCL_PARAM  PUKCLParam;
static PPUKCL_PARAM pvPUKCLParam = &PUKCLParam;

/* ------------------------------------------------------------------ */
/* P-256 sizes                                                        */
/* ------------------------------------------------------------------ */
#define P256_FIELD_BYTES  32u
#define P256_MOD_SIZE     P256_FIELD_BYTES      /* u2ModLength */
#define P256_OP           (P256_FIELD_BYTES + 4u)  /* 36 bytes per operand */

/* ------------------------------------------------------------------ */
/* Crypto RAM addresses                                               */
/* ------------------------------------------------------------------ */
#define CRAM_FAR   ((uint8_t *)(MSB_EXTENT_CRYPTORAM | nu1CRYPTORAM_BASE))
#define NEAR(off)  ((nu1)(nu1CRYPTORAM_BASE + (off)))

/* ---- MUL layout -------------------------------------------------- */
enum {
    OFF_MODULO   = 0,
    OFF_CNS      = OFF_MODULO + P256_OP,
    OFF_POINT_X  = OFF_CNS + P256_MOD_SIZE + 12,
    OFF_POINT_Y  = OFF_POINT_X + P256_OP,
    OFF_POINT_Z  = OFF_POINT_Y + P256_OP,
    OFF_A_CURVE  = OFF_POINT_Z + P256_OP + 8,
    OFF_SCALAR   = OFF_A_CURVE + P256_OP,
    OFF_ORDER    = OFF_SCALAR + P256_OP,
    OFF_WKSP     = OFF_ORDER + P256_OP
};

/* ---- SIGN layout ------------------------------------------------- */
enum {
    SIGN_MODULO    = 0,
    SIGN_CNS       = SIGN_MODULO + P256_OP,
    SIGN_POINT_A_X = SIGN_CNS + P256_MOD_SIZE + 12,
    SIGN_POINT_A_Y = SIGN_POINT_A_X + P256_OP,
    SIGN_POINT_A_Z = SIGN_POINT_A_Y + P256_OP,
    SIGN_A_CURVE   = SIGN_POINT_A_Z + P256_OP,
    SIGN_PRIVKEY   = SIGN_A_CURVE + P256_OP,
    SIGN_SCALAR    = SIGN_PRIVKEY + P256_OP,
    SIGN_ORDER     = SIGN_SCALAR + P256_OP,
    SIGN_HASH      = SIGN_ORDER + P256_OP,
    SIGN_WKSP      = SIGN_HASH + P256_OP
};

/* ---- VERIFY layout ----------------------------------------------- */
enum {
    VERIF_MODULO    = 0,
    VERIF_CNS       = VERIF_MODULO + P256_OP,
    VERIF_ORDER     = VERIF_CNS + P256_MOD_SIZE + 12,
    VERIF_SIG       = VERIF_ORDER + P256_MOD_SIZE + 12,
    VERIF_HASH      = VERIF_SIG + 2 * P256_OP,
    VERIF_POINT_A_X = VERIF_HASH + P256_OP,
    VERIF_POINT_A_Y = VERIF_POINT_A_X + P256_OP,
    VERIF_POINT_A_Z = VERIF_POINT_A_Y + P256_OP,
    VERIF_PUBKEY_X  = VERIF_POINT_A_Z + P256_OP,
    VERIF_PUBKEY_Y  = VERIF_PUBKEY_X + P256_OP,
    VERIF_PUBKEY_Z  = VERIF_PUBKEY_Y + P256_OP,
    VERIF_A_CURVE   = VERIF_PUBKEY_Z + P256_OP,
    VERIF_WKSP      = VERIF_A_CURVE + P256_OP
};

/* ------------------------------------------------------------------ */
/* P-256 curve constants (big-endian, 4-byte zero MSB prefix)         */
/* Source: Microchip Harmony crypto_v4                                */
/* ------------------------------------------------------------------ */

static const uint8_t p256_modulo[36] = {
    0x00,0x00,0x00,0x00, 0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
    0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF
};

static const uint8_t p256_a[36] = {
    0x00,0x00,0x00,0x00, 0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
    0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFC
};

static const uint8_t p256_gx[36] = {
    0x00,0x00,0x00,0x00, 0x6b,0x17,0xd1,0xf2, 0xe1,0x2c,0x42,0x47,
    0xf8,0xbc,0xe6,0xe5, 0x63,0xa4,0x40,0xf2, 0x77,0x03,0x7d,0x81,
    0x2d,0xeb,0x33,0xa0, 0xf4,0xa1,0x39,0x45, 0xd8,0x98,0xc2,0x96
};

static const uint8_t p256_gy[36] = {
    0x00,0x00,0x00,0x00, 0x4f,0xe3,0x42,0xe2, 0xfe,0x1a,0x7f,0x9b,
    0x8e,0xe7,0xeb,0x4a, 0x7c,0x0f,0x9e,0x16, 0x2b,0xce,0x33,0x57,
    0x6b,0x31,0x5e,0xce, 0xcb,0xb6,0x40,0x68, 0x37,0xbf,0x51,0xf5
};

static const uint8_t p256_one[36] = {
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x01
};

static const uint8_t p256_order[36] = {
    0x00,0x00,0x00,0x00, 0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x00,
    0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF, 0xBC,0xE6,0xFA,0xAD,
    0xA7,0x17,0x9E,0x84, 0xF3,0xB9,0xCA,0xC2, 0xFC,0x63,0x25,0x51
};

static const uint8_t p256_cns[40] = {
    0x00,0x00,0x00,0x01, 0x00,0x00,0x00,0x00, 0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFE, 0xFF,0xFF,0xFF,0xFE, 0xFF,0xFF,0xFF,0xFE,
    0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x03,
    0x00,0x00,0x00,0x05
};

/* ------------------------------------------------------------------ */
/* Self-test checksums                                                */
/* ------------------------------------------------------------------ */
#define PUKCL_SELFTEST_CHECKSUM1   0x6E70DDD2u
#define PUKCL_SELFTEST_CHECKSUM2   0x25C8D64Fu

/* ------------------------------------------------------------------ */
/* State: 0 = not done, 1 = OK, <0 = failed                          */
/* ------------------------------------------------------------------ */
static int initState = 0;

/* Diagnostic: last PUKCL error code (visible via debugger) */
volatile int lastEcdsaError = 0;

/* ------------------------------------------------------------------ */
/* CRAM helpers.  PUKCL stores integers LSB-first (datasheet §43.3).  */
/* Our constants and mbedTLS data are big-endian, so we reverse.      */
/* ------------------------------------------------------------------ */

static void cramZero(unsigned offset, unsigned len)
{
    memset(CRAM_FAR + offset, 0, len);
}

static void cramWriteLE(unsigned offset, const void *src, unsigned len)
{
    const uint8_t *s = (const uint8_t *)src;
    uint8_t *dst = CRAM_FAR + offset;
    unsigned i;
    for (i = 0; i < len; i++)
        dst[i] = s[len - 1 - i];
}

static void cramWriteOperand(unsigned offset, const uint8_t *be32)
{
    cramWriteLE(offset, be32, P256_FIELD_BYTES);
    cramZero(offset + P256_FIELD_BYTES, 4);
}

static void cramReadBE(unsigned offset, uint8_t *dst, unsigned len)
{
    const uint8_t *src = CRAM_FAR + offset;
    unsigned i;
    for (i = 0; i < len; i++)
        dst[i] = src[len - 1 - i];
}

/* ------------------------------------------------------------------ */
/* ecc_p256_init                                                      */
/* ------------------------------------------------------------------ */
int ecc_p256_init(void)
{
    if (initState != 0) return (initState > 0) ? 0 : -1;

    /* Enable PUKCC AHB clock (MCLK AHBMASK bit 20) */
    MCLK->AHBMASK.reg |= (1u << 20);

    /* Wait for crypto RAM clear to finish */
    while ((PUKCCSR & BIT_PUKCCSR_CLRRAM_BUSY) != 0) { }

    /* Run self-test */
    memset(&PUKCLParam, 0, sizeof(PUKCLParam));
    pvPUKCLParam = &PUKCLParam;
    vPUKCL_Process(SelfTest, pvPUKCLParam);

    if (PUKCL(u2Status) != PUKCL_OK) {
        initState = -1; return -1;
    }

    if (pvPUKCLParam->P.PUKCL_SelfTest.u4Version != PUKCL_VERSION) {
        initState = -2; return -2;
    }
    if (pvPUKCLParam->P.PUKCL_SelfTest.u4CheckNum1 != PUKCL_SELFTEST_CHECKSUM1) { initState = -3; return -3; }
    if (pvPUKCLParam->P.PUKCL_SelfTest.u4CheckNum2 != PUKCL_SELFTEST_CHECKSUM2) { initState = -4; return -4; }

    initState = 1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* ecc_p256_mul — R = k * P on secp256r1                              */
/* ------------------------------------------------------------------ */
int ecc_p256_mul(
    const uint8_t *scalar,
    const uint8_t *px, const uint8_t *py,
    uint8_t *rx, uint8_t *ry)
{
    if (initState <= 0) return -1;

    /* Load constants (reversed to LSB-first) */
    cramWriteLE(OFF_MODULO, p256_modulo, sizeof(p256_modulo));
    cramWriteLE(OFF_CNS, p256_cns, sizeof(p256_cns));
    cramWriteLE(OFF_A_CURVE, p256_a, sizeof(p256_a));
    cramWriteLE(OFF_ORDER, p256_order, sizeof(p256_order));

    /* Load point */
    if (px != NULL) {
        cramWriteOperand(OFF_POINT_X, px);
        cramWriteOperand(OFF_POINT_Y, py);
    } else {
        cramWriteLE(OFF_POINT_X, p256_gx, sizeof(p256_gx));
        cramWriteLE(OFF_POINT_Y, p256_gy, sizeof(p256_gy));
    }
    cramWriteLE(OFF_POINT_Z, p256_one, sizeof(p256_one));

    /* Load scalar */
    cramWriteOperand(OFF_SCALAR, scalar);

    /* Clear workspace and gap */
    cramZero(OFF_WKSP, 0x1000 - OFF_WKSP);
    cramZero(OFF_POINT_Z + P256_OP, 8);

    /* ZpEccMulFast */
    memset(&PUKCLParam, 0, sizeof(PUKCLParam));
    PUKCL_ZpEccMul(nu1ModBase)   = NEAR(OFF_MODULO);
    PUKCL_ZpEccMul(nu1CnsBase)   = NEAR(OFF_CNS);
    PUKCL_ZpEccMul(nu1PointBase) = NEAR(OFF_POINT_X);
    PUKCL_ZpEccMul(nu1ABase)     = NEAR(OFF_A_CURVE);
    PUKCL_ZpEccMul(nu1KBase)     = NEAR(OFF_SCALAR);
    PUKCL_ZpEccMul(nu1Workspace) = NEAR(OFF_WKSP);
    PUKCL_ZpEccMul(u2ModLength)  = P256_MOD_SIZE;
    PUKCL_ZpEccMul(u2KLength)    = P256_MOD_SIZE;

    vPUKCL_Process(ZpEccMulFast, pvPUKCLParam);

    if (PUKCL(u2Status) != PUKCL_OK) return -10 - PUKCL(u2Status);

    /* Projective to affine */
    memset(&PUKCLParam, 0, sizeof(PUKCLParam));
    PUKCL_ZpEcConvProjToAffine(nu1ModBase)   = NEAR(OFF_MODULO);
    PUKCL_ZpEcConvProjToAffine(nu1CnsBase)   = NEAR(OFF_CNS);
    PUKCL_ZpEcConvProjToAffine(nu1PointABase) = NEAR(OFF_POINT_X);
    PUKCL_ZpEcConvProjToAffine(u2ModLength)  = P256_MOD_SIZE;
    PUKCL_ZpEcConvProjToAffine(nu1Workspace) = NEAR(OFF_WKSP);

    vPUKCL_Process(ZpEcConvProjToAffine, pvPUKCLParam);

    if (PUKCL(u2Status) != PUKCL_OK) return -20 - PUKCL(u2Status);

    /* Read result */
    cramReadBE(OFF_POINT_X, rx, P256_FIELD_BYTES);
    cramReadBE(OFF_POINT_Y, ry, P256_FIELD_BYTES);

    return 0;
}

/* ------------------------------------------------------------------ */
/* ecc_p256_ecdsa_sign                                                */
/* ------------------------------------------------------------------ */
int ecc_p256_ecdsa_sign(
    const uint8_t *hash,
    const uint8_t *private_key,
    const uint8_t *nonce_k,
    uint8_t *sig_r, uint8_t *sig_s)
{
    if (initState <= 0) return -1;

    cramZero(0, 0x1000);

    cramWriteLE(SIGN_MODULO, p256_modulo, sizeof(p256_modulo));
    cramWriteLE(SIGN_CNS, p256_cns, sizeof(p256_cns));
    cramWriteLE(SIGN_A_CURVE, p256_a, sizeof(p256_a));
    cramWriteLE(SIGN_ORDER, p256_order, sizeof(p256_order));

    cramWriteLE(SIGN_POINT_A_X, p256_gx, sizeof(p256_gx));
    cramWriteLE(SIGN_POINT_A_Y, p256_gy, sizeof(p256_gy));
    cramWriteLE(SIGN_POINT_A_Z, p256_one, sizeof(p256_one));

    cramWriteOperand(SIGN_PRIVKEY, private_key);
    cramWriteOperand(SIGN_SCALAR, nonce_k);
    cramWriteOperand(SIGN_HASH, hash);

    memset(&PUKCLParam, 0, sizeof(PUKCLParam));
    PUKCL_ZpEcDsaGenerate(nu1ModBase)        = NEAR(SIGN_MODULO);
    PUKCL_ZpEcDsaGenerate(nu1CnsBase)        = NEAR(SIGN_CNS);
    PUKCL_ZpEcDsaGenerate(nu1PointABase)     = NEAR(SIGN_POINT_A_X);
    PUKCL_ZpEcDsaGenerate(nu1PrivateKey)     = NEAR(SIGN_PRIVKEY);
    PUKCL_ZpEcDsaGenerate(nu1ScalarNumber)   = NEAR(SIGN_SCALAR);
    PUKCL_ZpEcDsaGenerate(nu1OrderPointBase) = NEAR(SIGN_ORDER);
    PUKCL_ZpEcDsaGenerate(nu1ABase)          = NEAR(SIGN_A_CURVE);
    PUKCL_ZpEcDsaGenerate(nu1Workspace)      = NEAR(SIGN_WKSP);
    PUKCL_ZpEcDsaGenerate(nu1HashBase)       = NEAR(SIGN_HASH);
    PUKCL_ZpEcDsaGenerate(u2ModLength)       = P256_MOD_SIZE;
    PUKCL_ZpEcDsaGenerate(u2ScalarLength)    = P256_MOD_SIZE;

    vPUKCL_Process(ZpEcDsaGenerateFast, pvPUKCLParam);

    if (PUKCL(u2Status) != PUKCL_OK) {
        lastEcdsaError = PUKCL(u2Status);
        return -30 - PUKCL(u2Status);
    }

    cramReadBE(SIGN_POINT_A_X, sig_r, P256_FIELD_BYTES);
    cramReadBE(SIGN_POINT_A_Y, sig_s, P256_FIELD_BYTES);

    return 0;
}

/* ------------------------------------------------------------------ */
/* ecc_p256_ecdsa_verify                                              */
/* ------------------------------------------------------------------ */
int ecc_p256_ecdsa_verify(
    const uint8_t *hash,
    const uint8_t *pub_x, const uint8_t *pub_y,
    const uint8_t *sig_r, const uint8_t *sig_s)
{
    if (initState <= 0) return -1;

    cramZero(0, 0x1000);

    cramWriteLE(VERIF_MODULO, p256_modulo, sizeof(p256_modulo));
    cramWriteLE(VERIF_CNS, p256_cns, sizeof(p256_cns));
    cramWriteLE(VERIF_A_CURVE, p256_a, sizeof(p256_a));
    cramWriteLE(VERIF_ORDER, p256_order, sizeof(p256_order));

    cramWriteOperand(VERIF_SIG, sig_r);
    cramWriteOperand(VERIF_SIG + P256_OP, sig_s);

    cramWriteOperand(VERIF_HASH, hash);

    cramWriteLE(VERIF_POINT_A_X, p256_gx, sizeof(p256_gx));
    cramWriteLE(VERIF_POINT_A_Y, p256_gy, sizeof(p256_gy));
    cramWriteLE(VERIF_POINT_A_Z, p256_one, sizeof(p256_one));

    cramWriteOperand(VERIF_PUBKEY_X, pub_x);
    cramWriteOperand(VERIF_PUBKEY_Y, pub_y);
    cramWriteLE(VERIF_PUBKEY_Z, p256_one, sizeof(p256_one));

    memset(&PUKCLParam, 0, sizeof(PUKCLParam));
    PUKCL_ZpEcDsaVerify(nu1ModBase)          = NEAR(VERIF_MODULO);
    PUKCL_ZpEcDsaVerify(nu1CnsBase)          = NEAR(VERIF_CNS);
    PUKCL_ZpEcDsaVerify(nu1PointABase)       = NEAR(VERIF_POINT_A_X);
    PUKCL_ZpEcDsaVerify(nu1PointPublicKeyGen) = NEAR(VERIF_PUBKEY_X);
    PUKCL_ZpEcDsaVerify(nu1PointSignature)   = NEAR(VERIF_SIG);
    PUKCL_ZpEcDsaVerify(nu1OrderPointBase)   = NEAR(VERIF_ORDER);
    PUKCL_ZpEcDsaVerify(nu1ABase)            = NEAR(VERIF_A_CURVE);
    PUKCL_ZpEcDsaVerify(nu1Workspace)        = NEAR(VERIF_WKSP);
    PUKCL_ZpEcDsaVerify(nu1HashBase)         = NEAR(VERIF_HASH);
    PUKCL_ZpEcDsaVerify(u2ModLength)         = P256_MOD_SIZE;
    PUKCL_ZpEcDsaVerify(u2ScalarLength)      = P256_MOD_SIZE;

    vPUKCL_Process(ZpEcDsaVerifyFast, pvPUKCLParam);

    if (PUKCL(u2Status) != PUKCL_OK) return -40 - PUKCL(u2Status);

    return 0;
}

#endif /* __SAME54P20A__ || __SAME51N19A__ */
