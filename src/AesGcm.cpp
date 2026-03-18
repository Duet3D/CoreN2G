/*
 * AesGcm.cpp
 *
 * Hardware AES-GCM driver for Atmel/Microchip SAM MCUs.
 * Provides one-shot aes_gcm_encrypt() and aes_gcm_decrypt() using the
 * on-chip AES peripheral in GCM mode.
 *
 * Supports SAME70 and SAME5x (SAME54/SAME51) chip families.
 * Uses direct register access (same approach as TrngEntropy.cpp).
 *
 * Thread safety: the AES peripheral is only used from the networking
 * task, so no locking is needed. Do not call from multiple tasks.
 */

#include <cstring>
#include <cstdint>

// ============================================================
// SAME70 AES-GCM — peripheral at 0x4006C000
// ============================================================
#if defined(__SAME70Q20B__) || defined(__SAME70Q21B__) || defined(__SAME70N20B__) || defined(__SAME70N21B__)

#define AES_BASE				0x4006C000U

#define AES_CR					(*(volatile uint32_t *)(AES_BASE + 0x00U))
#define AES_MR					(*(volatile uint32_t *)(AES_BASE + 0x04U))
#define AES_IER					(*(volatile uint32_t *)(AES_BASE + 0x10U))
#define AES_IDR					(*(volatile uint32_t *)(AES_BASE + 0x14U))
#define AES_ISR					(*(volatile const uint32_t *)(AES_BASE + 0x1CU))
#define AES_KEYWR				((volatile uint32_t *)(AES_BASE + 0x20U))
#define AES_IDATAR				((volatile uint32_t *)(AES_BASE + 0x40U))
#define AES_ODATAR				((volatile const uint32_t *)(AES_BASE + 0x50U))
#define AES_IVR					((volatile uint32_t *)(AES_BASE + 0x60U))
#define AES_AADLENR				(*(volatile uint32_t *)(AES_BASE + 0x70U))
#define AES_CLENR				(*(volatile uint32_t *)(AES_BASE + 0x74U))
#define AES_TAGR				((volatile const uint32_t *)(AES_BASE + 0x88U))

// CR bits
#define AES_CR_START			(1U << 0)
#define AES_CR_SWRST			(1U << 8)

// MR bits
#define AES_MR_CIPHER			(1U << 0)			// 1 = encrypt, 0 = decrypt
#define AES_MR_GTAGEN			(1U << 1)			// GCM auto-tag generation
#define AES_MR_SMOD_AUTO		(1U << 8)			// auto-start on IDATAR write
#define AES_MR_OPMOD_GCM		(5U << 12)			// GCM mode
#define AES_MR_KEYSIZE_128		(0U << 10)
#define AES_MR_KEYSIZE_192		(1U << 10)
#define AES_MR_KEYSIZE_256		(2U << 10)

// ISR bits
#define AES_ISR_DATRDY			(1U << 0)
#define AES_ISR_TAGRDY			(1U << 16)

// PMC for clock enable
#define PMC_BASE				0x400E0600U
#define PMC_PCR					(*(volatile uint32_t *)(PMC_BASE + 0x010CU))
#define PMC_PCR_EN				(1U << 28)
#define PMC_PCR_CMD				(1U << 12)
#define ID_AES					56U

static void aes_hw_init() noexcept
{
	// Enable peripheral clock
	PMC_PCR = (ID_AES & 0x7FU);
	PMC_PCR = PMC_PCR | PMC_PCR_EN | PMC_PCR_CMD;

	// Software reset
	AES_CR = AES_CR_SWRST;
}

static uint32_t aes_keysize_bits_to_mr(size_t key_bits) noexcept
{
	switch (key_bits)
	{
	case 128: return AES_MR_KEYSIZE_128;
	case 192: return AES_MR_KEYSIZE_192;
	case 256: return AES_MR_KEYSIZE_256;
	default:  return 0xFFFFFFFFU;
	}
}

static void aes_write_key(const uint8_t *key, size_t key_len) noexcept
{
	const size_t words = key_len / 4;
	for (size_t i = 0; i < words; i++)
	{
		uint32_t w;
		memcpy(&w, key + i * 4, 4);
		AES_KEYWR[i] = w;
	}
}

static void aes_write_iv(const uint8_t *iv) noexcept
{
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w;
		memcpy(&w, iv + i * 4, 4);
		AES_IVR[i] = w;
	}
}

static void aes_write_block(const uint8_t *data) noexcept
{
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w;
		memcpy(&w, data + i * 4, 4);
		AES_IDATAR[i] = w;
	}
}

static void aes_read_block(uint8_t *data) noexcept
{
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w = AES_ODATAR[i];
		memcpy(data + i * 4, &w, 4);
	}
}

static inline void aes_wait_datrdy() noexcept
{
	while ((AES_ISR & AES_ISR_DATRDY) == 0) {}
}

static void aes_read_tag(uint8_t *tag, size_t tag_len) noexcept
{
	// Wait for tag ready
	while ((AES_ISR & AES_ISR_TAGRDY) == 0) {}
	uint8_t full_tag[16];
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w = AES_TAGR[i];
		memcpy(full_tag + i * 4, &w, 4);
	}
	memcpy(tag, full_tag, (tag_len < 16) ? tag_len : 16);
}

// ============================================================
// SAME5x (SAME54 / SAME51) AES-GCM — peripheral at 0x42002400
// ============================================================
#elif defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAME51P20A__) || defined(__SAME54N20A__)

#define AES_BASE				0x42002400U

#define AES_CTRLA				(*(volatile uint32_t *)(AES_BASE + 0x00U))
#define AES_CTRLB				(*(volatile uint8_t  *)(AES_BASE + 0x04U))
#define AES_INTENCLR			(*(volatile uint8_t  *)(AES_BASE + 0x05U))
#define AES_INTENSET			(*(volatile uint8_t  *)(AES_BASE + 0x06U))
#define AES_INTFLAG				(*(volatile uint8_t  *)(AES_BASE + 0x07U))
#define AES_DATABUFPTR			(*(volatile uint8_t  *)(AES_BASE + 0x08U))
#define AES_KEYWORD				((volatile uint32_t *)(AES_BASE + 0x0CU))
#define AES_INDATA				(*(volatile uint32_t *)(AES_BASE + 0x38U))
#define AES_INTVECTV			((volatile uint32_t *)(AES_BASE + 0x3CU))
#define AES_HASHKEY				((volatile uint32_t *)(AES_BASE + 0x5CU))
#define AES_GHASH				((volatile uint32_t *)(AES_BASE + 0x6CU))
#define AES_CIPLEN				(*(volatile uint32_t *)(AES_BASE + 0x80U))

// CTRLA bits
#define AES_CTRLA_SWRST			(1U << 0)
#define AES_CTRLA_ENABLE		(1U << 1)
#define AES_CTRLA_AESMODE_GCM	(6U << 2)
#define AES_CTRLA_CIPHER_ENC	(1U << 10)
#define AES_CTRLA_STARTMODE_AUTO (1U << 11)
#define AES_CTRLA_KEYSIZE_128	(0U << 8)
#define AES_CTRLA_KEYSIZE_192	(1U << 8)
#define AES_CTRLA_KEYSIZE_256	(2U << 8)

// CTRLB bits
#define AES_CTRLB_START			(1U << 0)
#define AES_CTRLB_NEWMSG		(1U << 1)
#define AES_CTRLB_EOM			(1U << 2)
#define AES_CTRLB_GFMUL		(1U << 3)

// INTFLAG bits
#define AES_INTFLAG_ENCCMP		(1U << 0)
#define AES_INTFLAG_GFMCMP		(1U << 1)

// MCLK for clock gating
#define MCLK_BASE				0x40000800U
#define MCLK_APBCMASK			(*(volatile uint32_t *)(MCLK_BASE + 0x20U))
#define MCLK_APBCMASK_AES		(1U << 9)

static void aes_hw_init() noexcept
{
	// Enable APBC clock for AES
	MCLK_APBCMASK |= MCLK_APBCMASK_AES;

	// Software reset
	AES_CTRLA = AES_CTRLA_SWRST;
	while (AES_CTRLA & AES_CTRLA_SWRST) {}
}

static uint32_t aes_keysize_bits_to_ctrla(size_t key_bits) noexcept
{
	switch (key_bits)
	{
	case 128: return AES_CTRLA_KEYSIZE_128;
	case 192: return AES_CTRLA_KEYSIZE_192;
	case 256: return AES_CTRLA_KEYSIZE_256;
	default:  return 0xFFFFFFFFU;
	}
}

static void aes_write_key(const uint8_t *key, size_t key_len) noexcept
{
	const size_t words = key_len / 4;
	for (size_t i = 0; i < words; i++)
	{
		uint32_t w;
		memcpy(&w, key + i * 4, 4);
		AES_KEYWORD[i] = w;
	}
}

static void aes_write_iv(const uint8_t *iv) noexcept
{
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w;
		memcpy(&w, iv + i * 4, 4);
		AES_INTVECTV[i] = w;
	}
}

static inline void aes_wait_enccmp() noexcept
{
	while ((AES_INTFLAG & AES_INTFLAG_ENCCMP) == 0) {}
}

#else
#define AES_GCM_NOT_AVAILABLE	1
#endif

// ============================================================
// Common one-shot AES-GCM API
// ============================================================

#if !defined(AES_GCM_NOT_AVAILABLE)

static bool aes_initialised = false;

// ---- SAME70 implementation ----
#if defined(__SAME70Q20B__) || defined(__SAME70Q21B__) || defined(__SAME70N20B__) || defined(__SAME70N21B__)

static int aes_gcm_process(
	bool encrypt,
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *input, size_t input_len,
	uint8_t *output,
	uint8_t *tag, size_t tag_len) noexcept
{
	if (iv_len != 12 || tag_len > 16)
		return -1;

	const size_t key_bits = key_len * 8;
	const uint32_t ks = aes_keysize_bits_to_mr(key_bits);
	if (ks == 0xFFFFFFFFU)
		return -1;

	if (!aes_initialised)
	{
		aes_hw_init();
		aes_initialised = true;
	}

	// Software reset to clear state
	AES_CR = AES_CR_SWRST;

	// Configure: GCM mode, auto-start, tag generation, encrypt/decrypt
	uint32_t mr = AES_MR_OPMOD_GCM | AES_MR_SMOD_AUTO | AES_MR_GTAGEN | ks;
	if (encrypt)
		mr |= AES_MR_CIPHER;
	AES_MR = mr;

	// Write key
	aes_write_key(key, key_len);

	// Write IV: GCM uses 12-byte IV + 32-bit counter starting at 1
	// The hardware expects a 16-byte IV with counter = 2 for data blocks.
	// We write the 12-byte nonce padded with 0x00000001 (big-endian counter).
	uint8_t iv_block[16];
	memcpy(iv_block, iv, 12);
	iv_block[12] = 0;
	iv_block[13] = 0;
	iv_block[14] = 0;
	iv_block[15] = 1;
	aes_write_iv(iv_block);

	// Set AAD and ciphertext/plaintext lengths
	AES_AADLENR = (uint32_t)aad_len;
	AES_CLENR = (uint32_t)input_len;

	// Process AAD in 16-byte blocks
	size_t pos = 0;
	while (pos + 16 <= aad_len)
	{
		aes_write_block(aad + pos);
		aes_wait_datrdy();
		pos += 16;
	}

	// Partial AAD block (zero-padded)
	if (pos < aad_len)
	{
		uint8_t pad[16] = {};
		memcpy(pad, aad + pos, aad_len - pos);
		aes_write_block(pad);
		aes_wait_datrdy();
	}

	// Process data in 16-byte blocks
	pos = 0;
	while (pos + 16 <= input_len)
	{
		aes_write_block(input + pos);
		aes_wait_datrdy();
		aes_read_block(output + pos);
		pos += 16;
	}

	// Partial data block (zero-padded input, truncated output)
	if (pos < input_len)
	{
		uint8_t pad_in[16] = {};
		uint8_t pad_out[16];
		const size_t rem = input_len - pos;
		memcpy(pad_in, input + pos, rem);
		aes_write_block(pad_in);
		aes_wait_datrdy();
		aes_read_block(pad_out);
		memcpy(output + pos, pad_out, rem);
	}

	// Read authentication tag
	aes_read_tag(tag, tag_len);
	return 0;
}

// ---- SAME5x implementation ----
#elif defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAME51P20A__) || defined(__SAME54N20A__)

static int aes_gcm_process(
	bool encrypt,
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *input, size_t input_len,
	uint8_t *output,
	uint8_t *tag, size_t tag_len) noexcept
{
	if (iv_len != 12 || tag_len > 16)
		return -1;

	const size_t key_bits = key_len * 8;
	const uint32_t ks = aes_keysize_bits_to_ctrla(key_bits);
	if (ks == 0xFFFFFFFFU)
		return -1;

	if (!aes_initialised)
	{
		aes_hw_init();
		aes_initialised = true;
	}

	// Disable then configure
	AES_CTRLA = 0;

	uint32_t ctrla = AES_CTRLA_ENABLE | AES_CTRLA_AESMODE_GCM | ks;
	if (encrypt)
		ctrla |= AES_CTRLA_CIPHER_ENC;
	AES_CTRLA = ctrla;

	// Write key
	aes_write_key(key, key_len);

	// Set cipher length
	AES_CIPLEN = (uint32_t)input_len;

	// Step 1: Generate H (hash key) — write IV=0, start with NEWMSG
	uint8_t zero_iv[16] = {};
	aes_write_iv(zero_iv);

	// Clear GHASH and HASHKEY
	for (int i = 0; i < 4; i++)
	{
		AES_GHASH[i] = 0;
		AES_HASHKEY[i] = 0;
	}

	// Trigger GFMUL to start H generation
	AES_CTRLB = AES_CTRLB_START | AES_CTRLB_NEWMSG;

	// Write zeroes to generate H
	AES_DATABUFPTR = 0;
	for (int i = 0; i < 4; i++)
	{
		AES_INDATA = 0;
	}
	aes_wait_enccmp();
	AES_INTFLAG = AES_INTFLAG_ENCCMP;

	// Read H from output into HASHKEY
	// On SAME5x GCM, the first encryption of zero block gives H
	// H is now in the internal state, we need to copy it to HASHKEY
	AES_DATABUFPTR = 0;
	for (int i = 0; i < 4; i++)
	{
		AES_HASHKEY[i] = AES_INDATA;	// Read output data via INDATA after ENCCMP
	}

	// Step 2: Set up real IV (J0 = IV || 0x00000001)
	uint8_t iv_block[16];
	memcpy(iv_block, iv, 12);
	iv_block[12] = 0;
	iv_block[13] = 0;
	iv_block[14] = 0;
	iv_block[15] = 2;		// Counter starts at 2 for data (J0+1)
	aes_write_iv(iv_block);

	// Clear GHASH for fresh start
	for (int i = 0; i < 4; i++)
	{
		AES_GHASH[i] = 0;
	}

	// Step 3: Process AAD
	AES_CTRLB = AES_CTRLB_NEWMSG;

	size_t pos = 0;
	while (pos + 16 <= aad_len)
	{
		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w;
			memcpy(&w, aad + pos + i * 4, 4);
			AES_INDATA = w;
		}
		// In GCM-AAD phase, hardware XORs with GHASH
		AES_CTRLB = AES_CTRLB_GFMUL;
		while ((AES_INTFLAG & AES_INTFLAG_GFMCMP) == 0) {}
		AES_INTFLAG = AES_INTFLAG_GFMCMP;
		pos += 16;
	}
	if (pos < aad_len)
	{
		uint8_t pad[16] = {};
		memcpy(pad, aad + pos, aad_len - pos);
		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w;
			memcpy(&w, pad + i * 4, 4);
			AES_INDATA = w;
		}
		AES_CTRLB = AES_CTRLB_GFMUL;
		while ((AES_INTFLAG & AES_INTFLAG_GFMCMP) == 0) {}
		AES_INTFLAG = AES_INTFLAG_GFMCMP;
	}

	// Step 4: Process data blocks
	pos = 0;
	while (pos + 16 <= input_len)
	{
		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w;
			memcpy(&w, input + pos + i * 4, 4);
			AES_INDATA = w;
		}
		AES_CTRLB = AES_CTRLB_START;
		aes_wait_enccmp();
		AES_INTFLAG = AES_INTFLAG_ENCCMP;

		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w = AES_INDATA;
			memcpy(output + pos + i * 4, &w, 4);
		}

		// GHASH update with ciphertext
		AES_CTRLB = AES_CTRLB_GFMUL;
		while ((AES_INTFLAG & AES_INTFLAG_GFMCMP) == 0) {}
		AES_INTFLAG = AES_INTFLAG_GFMCMP;

		pos += 16;
	}

	// Partial data block
	if (pos < input_len)
	{
		uint8_t pad_in[16] = {};
		uint8_t pad_out[16];
		const size_t rem = input_len - pos;
		memcpy(pad_in, input + pos, rem);

		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w;
			memcpy(&w, pad_in + i * 4, 4);
			AES_INDATA = w;
		}
		AES_CTRLB = AES_CTRLB_START;
		aes_wait_enccmp();
		AES_INTFLAG = AES_INTFLAG_ENCCMP;

		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w = AES_INDATA;
			memcpy(pad_out + i * 4, &w, 4);
		}
		memcpy(output + pos, pad_out, rem);

		AES_CTRLB = AES_CTRLB_GFMUL;
		while ((AES_INTFLAG & AES_INTFLAG_GFMCMP) == 0) {}
		AES_INTFLAG = AES_INTFLAG_GFMCMP;
	}

	// Step 5: Final GHASH — lengths block
	{
		uint8_t len_block[16] = {};
		// AAD length in bits (big-endian, 64-bit)
		const uint64_t aad_bits = (uint64_t)aad_len * 8;
		len_block[0] = (uint8_t)(aad_bits >> 56);
		len_block[1] = (uint8_t)(aad_bits >> 48);
		len_block[2] = (uint8_t)(aad_bits >> 40);
		len_block[3] = (uint8_t)(aad_bits >> 32);
		len_block[4] = (uint8_t)(aad_bits >> 24);
		len_block[5] = (uint8_t)(aad_bits >> 16);
		len_block[6] = (uint8_t)(aad_bits >> 8);
		len_block[7] = (uint8_t)(aad_bits);
		// Ciphertext length in bits (big-endian, 64-bit)
		const uint64_t ct_bits = (uint64_t)input_len * 8;
		len_block[8]  = (uint8_t)(ct_bits >> 56);
		len_block[9]  = (uint8_t)(ct_bits >> 48);
		len_block[10] = (uint8_t)(ct_bits >> 40);
		len_block[11] = (uint8_t)(ct_bits >> 32);
		len_block[12] = (uint8_t)(ct_bits >> 24);
		len_block[13] = (uint8_t)(ct_bits >> 16);
		len_block[14] = (uint8_t)(ct_bits >> 8);
		len_block[15] = (uint8_t)(ct_bits);

		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w;
			memcpy(&w, len_block + i * 4, 4);
			AES_INDATA = w;
		}
		AES_CTRLB = AES_CTRLB_GFMUL;
		while ((AES_INTFLAG & AES_INTFLAG_GFMCMP) == 0) {}
		AES_INTFLAG = AES_INTFLAG_GFMCMP;
	}

	// Step 6: Encrypt J0 (IV || 0x00000001) to get tag mask, XOR with GHASH
	{
		uint8_t j0[16];
		memcpy(j0, iv, 12);
		j0[12] = 0;
		j0[13] = 0;
		j0[14] = 0;
		j0[15] = 1;

		// Switch to CTR mode briefly to encrypt J0
		AES_CTRLA = 0;
		ctrla = AES_CTRLA_ENABLE | (4U << 2) | ks | AES_CTRLA_CIPHER_ENC;	// CTR mode
		AES_CTRLA = ctrla;
		aes_write_key(key, key_len);
		aes_write_iv(j0);

		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			AES_INDATA = 0;
		}
		AES_CTRLB = AES_CTRLB_START | AES_CTRLB_NEWMSG;
		aes_wait_enccmp();
		AES_INTFLAG = AES_INTFLAG_ENCCMP;

		// Read E(K, J0)
		uint8_t ej0[16];
		AES_DATABUFPTR = 0;
		for (int i = 0; i < 4; i++)
		{
			uint32_t w = AES_INDATA;
			memcpy(ej0 + i * 4, &w, 4);
		}

		// Tag = GHASH XOR E(K, J0)
		uint8_t full_tag[16];
		for (int i = 0; i < 4; i++)
		{
			uint32_t gh = AES_GHASH[i];
			uint32_t ej;
			memcpy(&ej, ej0 + i * 4, 4);
			uint32_t t = gh ^ ej;
			memcpy(full_tag + i * 4, &t, 4);
		}
		memcpy(tag, full_tag, (tag_len < 16) ? tag_len : 16);
	}

	return 0;
}

#endif // chip selection

#endif // !AES_GCM_NOT_AVAILABLE

// ============================================================
// Public C API
// ============================================================

extern "C" int aes_gcm_encrypt(
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *plaintext, size_t plaintext_len,
	uint8_t *ciphertext,
	uint8_t *tag, size_t tag_len) noexcept
{
#if defined(AES_GCM_NOT_AVAILABLE)
	(void)key; (void)key_len; (void)iv; (void)iv_len;
	(void)aad; (void)aad_len; (void)plaintext; (void)plaintext_len;
	(void)ciphertext; (void)tag; (void)tag_len;
	return -1;
#else
	return aes_gcm_process(true, key, key_len, iv, iv_len,
					   aad, aad_len, plaintext, plaintext_len,
					   ciphertext, tag, tag_len);
#endif
}

extern "C" int aes_gcm_decrypt(
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *ciphertext, size_t ciphertext_len,
	uint8_t *plaintext,
	const uint8_t *tag, size_t tag_len) noexcept
{
#if defined(AES_GCM_NOT_AVAILABLE)
	(void)key; (void)key_len; (void)iv; (void)iv_len;
	(void)aad; (void)aad_len; (void)ciphertext; (void)ciphertext_len;
	(void)plaintext; (void)tag; (void)tag_len;
	return -1;
#else
	uint8_t computed_tag[16];
	const size_t actual_tag_len = (tag_len < 16) ? tag_len : 16;

	int ret = aes_gcm_process(false, key, key_len, iv, iv_len,
							  aad, aad_len, ciphertext, ciphertext_len,
							  plaintext, computed_tag, actual_tag_len);

	if (ret != 0)
		return ret;

	// Constant-time tag comparison
	uint8_t diff = 0;
	for (size_t i = 0; i < actual_tag_len; i++)
	{
		diff |= computed_tag[i] ^ tag[i];
	}
	if (diff != 0)
	{
		// Authentication failed — zero plaintext to prevent misuse
		memset(plaintext, 0, ciphertext_len);
		return -2;
	}
	return 0;
#endif
}

// Decrypts ciphertext to plaintext and outputs the computed authentication tag
// without performing tag comparison.  Used by the mbedTLS GCM ALT wrapper so
// that mbedtls_gcm_auth_decrypt can do its own constant-time tag comparison.
extern "C" int aes_gcm_decrypt_and_tag(
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *ciphertext, size_t ciphertext_len,
	uint8_t *plaintext,
	uint8_t *tag, size_t tag_len) noexcept
{
#if defined(AES_GCM_NOT_AVAILABLE)
	(void)key; (void)key_len; (void)iv; (void)iv_len;
	(void)aad; (void)aad_len; (void)ciphertext; (void)ciphertext_len;
	(void)plaintext; (void)tag; (void)tag_len;
	return -1;
#else
	return aes_gcm_process(false, key, key_len, iv, iv_len,
						   aad, aad_len, ciphertext, ciphertext_len,
						   plaintext, tag, tag_len);
#endif
}
