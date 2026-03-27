/*
 * AesGcm.cpp
 *
 * Hardware AES-GCM driver for Atmel/Microchip SAM MCUs.
 * Provides one-shot aes_gcm_encrypt() and aes_gcm_decrypt() using the
 * on-chip AES peripheral in GCM mode.
 *
 * Supports SAME70 (using the ASF aes driver) and SAME5x (SAME54/SAME51).
 *
 * Thread safety: the AES peripheral is only used from the networking
 * task, so no locking is needed. Do not call from multiple tasks.
 */

#include <cstdint>
#include <cstring>

/* AesGcm and AesEcb share the same AES peripheral. Any AesGcm use invalidates
 * AesEcb's cached peripheral configuration so subsequent ECB calls reconfigure
 * safely. */
extern "C" void aes_ecb_invalidate_cache() noexcept;


// ============================================================
// SAME70 AES-GCM — uses ASF aes.h + pmc.h
// ============================================================
#if defined(__SAME70Q20B__) || defined(__SAME70Q21B__) || defined(__SAME70N20B__) || defined(__SAME70N21B__)

#include <aes/aes.h>
#include <pmc/pmc.h>

static void aes_hw_init() noexcept
{
	struct aes_config init_cfg;
	aes_get_config_defaults(&init_cfg);
	aes_init(AES, &init_cfg);
	aes_enable();
}

// Write byte-array key into the AES key registers (using the ASF function which
// reads key length from AES_MR, so we must call aes_set_config first).
static void aes_write_key_bytes(const uint8_t *key, size_t key_len) noexcept
{
	// aes_write_key expects 32-bit words; copy bytes into a word array first.
	uint32_t key_words[8] = {};
	memcpy(key_words, key, key_len);
	aes_write_key(AES, key_words);
}

// Write the 16-byte IV (as bytes) into AES_IVR.
static void aes_write_iv_bytes(const uint8_t *iv) noexcept
{
	uint32_t iv_words[4];
	memcpy(iv_words, iv, 16);
	aes_write_initvector(AES, iv_words);
}

// Write one 16-byte block (as bytes) to AES_IDATAR.
static void aes_write_block_bytes(const uint8_t *data) noexcept
{
	uint32_t words[4];
	memcpy(words, data, 16);
	aes_write_input_data(AES, words);
}

// Read one 16-byte block from AES_ODATAR.
static void aes_read_block_bytes(uint8_t *data) noexcept
{
	uint32_t words[4];
	aes_read_output_data(AES, words);
	memcpy(data, words, 16);
}

static inline void aes_wait_datrdy() noexcept
{
	while (!(aes_read_interrupt_status(AES) & AES_ISR_DATRDY)) {}
}

static void aes_read_tag_bytes(uint8_t *tag, size_t tag_len) noexcept
{
	while (!(aes_read_interrupt_status(AES) & AES_ISR_TAGRDY)) {}
	uint8_t full_tag[16];
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t w = aes_read_tag(AES, i);
		memcpy(full_tag + i * 4, &w, 4);
	}
	memcpy(tag, full_tag, (tag_len < 16) ? tag_len : 16);
}

// ============================================================
// SAME5x (SAME54 / SAME51) AES-GCM — uses ASF aes.h
//
// Follows the datasheet procedure (§42.6.3.1):
//   Phase 1: Hash key generation (ECB mode)
//   Phase 2: AAD processing (GCM mode, GFMUL)
//   Phase 3: Data processing (GCM mode, CTR + GHASH)
//   Phase 4: Length block GHASH finalization (GFMUL)
//   Phase 5: Tag generation (CTR mode, encrypt GHASH with J0)
//
// Decryption uses CIPHER=Decryption so the hardware GHASHes the
// input side (ciphertext), matching the SAME70 approach.
// ============================================================
#elif defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAME51P20A__) || defined(__SAME54N20A__)

#include <aes/aes.h>

static void aes_hw_init() noexcept
{
	struct aes_config init_cfg;
	aes_get_config_defaults(&init_cfg);
	aes_init(AES, &init_cfg);
	aes_enable();
}

static inline void aes_wait_datrdy() noexcept
{
	while (!(aes_read_interrupt_status(AES) & AES_ISR_DATRDY)) {}
}

static void aes_write_key_bytes(const uint8_t *key, size_t key_len) noexcept
{
	uint32_t key_words[8] = {};
	memcpy(key_words, key, key_len);
	aes_write_key(AES, key_words);
}

static void aes_write_iv_bytes(const uint8_t *iv) noexcept
{
	uint32_t iv_words[4];
	memcpy(iv_words, iv, 16);
	aes_write_initvector(AES, iv_words);
}

static void aes_write_block_bytes(const uint8_t *data) noexcept
{
	uint32_t words[4];
	memcpy(words, data, 16);
	aes_write_input_data(AES, words);
}

static void aes_read_block_bytes(uint8_t *data) noexcept
{
	uint32_t words[4];
	aes_read_output_data(AES, words);
	memcpy(data, words, 16);
}

static void aes_read_tag_bytes(uint8_t *tag, size_t tag_len) noexcept
{
	/* On SAME5x the GCM tag is produced by the CTR-mode step and read
	 * from INDATA, not from GHASH.  This helper is only used when the
	 * caller has already waited for ENCCMP after the CTR tag step. */
	uint8_t full_tag[16];
	aes_read_block_bytes(full_tag);
	memcpy(tag, full_tag, (tag_len < 16) ? tag_len : 16);
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
	enum aes_key_size ks;
	switch (key_bits)
	{
	case 128: ks = AES_KEY_SIZE_128; break;
	case 192: ks = AES_KEY_SIZE_192; break;
	case 256: ks = AES_KEY_SIZE_256; break;
	default:  return -1;
	}
	if (!aes_initialised)
	{
		aes_hw_init();
		aes_initialised = true;
	}

	// Use a fresh per-message config instead of carrying state in a shared
	// global config object.
	struct aes_config cfg;
	aes_get_config_defaults(&cfg);
	cfg.encrypt_mode = encrypt ? AES_ENCRYPTION : AES_DECRYPTION;
	cfg.key_size     = ks;
	cfg.start_mode   = AES_AUTO_START;
	cfg.opmode       = AES_GCM_MODE;
	cfg.cfb_size     = AES_CFB_SIZE_128;
	cfg.lod          = false;
	cfg.gtag_en      = true;
	cfg.processing_delay = 0;
	aes_set_config(AES, &cfg);

	// Write key — this also triggers GCMH (hash subkey) generation; wait for DATRDY
	aes_write_key_bytes(key, key_len);
	aes_wait_datrdy();

	// Build J0+1: 12-byte IV || counter=2 in big-endian.
	// The 32-bit counter word with value 2 in big-endian is bytes {0,0,0,2},
	// which when stored as a little-endian uint32_t equals 0x02000000.
	uint8_t iv_block[16];
	memcpy(iv_block, iv, 12);
	// Write {0x00, 0x00, 0x00, 0x02} as a big-endian counter word
	iv_block[12] = 0x00;
	iv_block[13] = 0x00;
	iv_block[14] = 0x00;
	iv_block[15] = 0x02;
	aes_write_iv_bytes(iv_block);

	// Set AAD length and cipher text/plain text length
	aes_write_authen_datalength(AES, (uint32_t)aad_len);
	aes_write_pctext_length(AES, (uint32_t)input_len);

	// Process AAD in 16-byte blocks
	size_t pos = 0;
	while (pos + 16 <= aad_len)
	{
		aes_write_block_bytes(aad + pos);
		aes_wait_datrdy();
		pos += 16;
	}

	// Partial AAD block (zero-padded)
	if (pos < aad_len)
	{
		uint8_t pad[16] = {};
		memcpy(pad, aad + pos, aad_len - pos);
		aes_write_block_bytes(pad);
		aes_wait_datrdy();
	}

	// Process data in 16-byte blocks
	pos = 0;
	while (pos + 16 <= input_len)
	{
		aes_write_block_bytes(input + pos);
		aes_wait_datrdy();
		if (output != nullptr)
		{
			aes_read_block_bytes(output + pos);
		}
		else
		{
			uint8_t discard[16];
			aes_read_block_bytes(discard);
		}
		pos += 16;
	}

	// Partial data block (zero-padded input, truncated output)
	if (pos < input_len)
	{
		uint8_t pad_in[16] = {};
		uint8_t pad_out[16];
		const size_t rem = input_len - pos;
		memcpy(pad_in, input + pos, rem);
		aes_write_block_bytes(pad_in);
		aes_wait_datrdy();
		aes_read_block_bytes(pad_out);
		if (output != nullptr)
		{
			memcpy(output + pos, pad_out, rem);
		}
	}

	// Read authentication tag
	aes_read_tag_bytes(tag, tag_len);
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
	enum aes_key_size ks;
	switch (key_bits)
	{
	case 128: ks = AES_KEY_SIZE_128; break;
	case 192: ks = AES_KEY_SIZE_192; break;
	case 256: ks = AES_KEY_SIZE_256; break;
	default:  return -1;
	}

	if (!aes_initialised)
	{
		aes_hw_init();
		aes_initialised = true;
	}

	// Software reset to guarantee clean peripheral state between
	// consecutive GCM operations — prevents internal state leakage.
	AES->CTRLA.reg = AES_CTRLA_SWRST;
	while (AES->CTRLA.reg & AES_CTRLA_SWRST) {}

	// === Phase 1: Hash Key Generation (ECB mode) ===
	// Per datasheet §42.6.3.1.1: ECB-encrypt zeros; hardware auto-populates
	// the HASHKEY register with the hash sub-key H.
	{
		struct aes_config cfg;
		aes_get_config_defaults(&cfg);
		cfg.encrypt_mode = AES_ENCRYPTION;
		cfg.key_size     = ks;
		cfg.start_mode   = AES_MANUAL_START;
		cfg.opmode       = AES_ECB_MODE;
		aes_set_config(AES, &cfg);

		AES->CIPLEN.reg = 0;
		aes_write_key_bytes(key, key_len);

		uint8_t zeros[16] = {};
		aes_write_block_bytes(zeros);
		aes_start(AES);
		aes_wait_datrdy();

		// Read and discard output to clear ENCCMP
		uint8_t discard[16];
		aes_read_block_bytes(discard);
	}

	// === Phase 2: Reconfigure for GCM mode ===
	{
		struct aes_config cfg;
		aes_get_config_defaults(&cfg);
		cfg.encrypt_mode = encrypt ? AES_ENCRYPTION : AES_DECRYPTION;
		cfg.key_size     = ks;
		cfg.start_mode   = AES_MANUAL_START;
		cfg.opmode       = AES_GCM_MODE;
		aes_set_config(AES, &cfg);

		aes_write_key_bytes(key, key_len);

		// Zero GHASH for fresh start (RW register, writable anytime)
		for (size_t i = 0; i < 4; i++)
		{
			AES->GHASH[i].reg = 0;
		}
	}

	// === Phase 3: AAD Processing (GFMUL) ===
	// Per datasheet §42.6.3.1.2: write data, then GFMUL|START in one write
	if (aad != nullptr && aad_len > 0)
	{
		size_t pos = 0;
		while (pos < aad_len)
		{
			uint8_t block[16] = {};
			const size_t n = (aad_len - pos >= 16) ? 16 : (aad_len - pos);
			memcpy(block, aad + pos, n);

			aes_write_block_bytes(block);
			AES->INTFLAG.reg = AES_INTFLAG_GFMCMP;
			AES->CTRLB.reg = AES_CTRLB_GFMUL | AES_CTRLB_START;
			while (!(AES->INTFLAG.reg & AES_INTFLAG_GFMCMP)) {}

			pos += 16;
		}
	}

	// === Phase 4: Data Processing ===
	{
		// J0+1: IV || counter=2 (big-endian)
		uint8_t iv_block[16];
		memcpy(iv_block, iv, 12);
		iv_block[12] = 0x00;
		iv_block[13] = 0x00;
		iv_block[14] = 0x00;
		iv_block[15] = 0x02;

		if (encrypt)
		{
			// Encrypt uses a two-pass approach: CTR-encrypt data, then
			// GFMUL the ciphertext into GHASH separately. This avoids
			// relying on the hardware's combined GCM encrypt which
			// produces wrong tags for certain records.

			// Save GHASH and HASHKEY — mode switches destroy them
			uint32_t ghash_save[4];
			uint32_t hashkey_save[4];
			for (size_t i = 0; i < 4; i++)
			{
				ghash_save[i] = AES->GHASH[i].reg;
				hashkey_save[i] = AES->HASHKEY[i].reg;
			}

			// --- Pass 1: CTR encrypt ---
			{
				struct aes_config cfg;
				aes_get_config_defaults(&cfg);
				cfg.encrypt_mode = AES_ENCRYPTION;
				cfg.key_size     = ks;
				cfg.start_mode   = AES_MANUAL_START;
				cfg.opmode       = AES_CTR_MODE;
				aes_set_config(AES, &cfg);
				aes_write_key_bytes(key, key_len);
				aes_write_iv_bytes(iv_block);

				size_t pos = 0;
				bool first = true;
				while (pos + 16 <= input_len)
				{
					aes_write_block_bytes(input + pos);
					AES->INTFLAG.reg = AES_INTFLAG_ENCCMP;
					uint8_t ctrlb = AES_CTRLB_START;
					if (first) ctrlb |= AES_CTRLB_NEWMSG;
					AES->CTRLB.reg = ctrlb;
					first = false;
					aes_wait_datrdy();
					aes_read_block_bytes(output + pos);
					pos += 16;
				}
				if (pos < input_len)
				{
					uint8_t pad_in[16] = {};
					const size_t rem = input_len - pos;
					memcpy(pad_in, input + pos, rem);
					aes_write_block_bytes(pad_in);
					AES->INTFLAG.reg = AES_INTFLAG_ENCCMP;
					uint8_t ctrlb = AES_CTRLB_START;
					if (first) ctrlb |= AES_CTRLB_NEWMSG;
					AES->CTRLB.reg = ctrlb;
					aes_wait_datrdy();
					uint8_t pad_out[16];
					aes_read_block_bytes(pad_out);
					memcpy(output + pos, pad_out, rem);
				}
			}

			// --- Pass 2: GFMUL ciphertext into GHASH ---
			{
				struct aes_config cfg;
				aes_get_config_defaults(&cfg);
				cfg.encrypt_mode = AES_ENCRYPTION;
				cfg.key_size     = ks;
				cfg.start_mode   = AES_MANUAL_START;
				cfg.opmode       = AES_GCM_MODE;
				aes_set_config(AES, &cfg);
				aes_write_key_bytes(key, key_len);

				// Restore GHASH and HASHKEY (destroyed by mode switches)
				for (size_t i = 0; i < 4; i++)
				{
					AES->GHASH[i].reg = ghash_save[i];
					AES->HASHKEY[i].reg = hashkey_save[i];
				}

				size_t pos = 0;
				while (pos < input_len)
				{
					uint8_t block[16] = {};
					const size_t n = (input_len - pos >= 16) ? 16 : (input_len - pos);
					memcpy(block, output + pos, n);
					aes_write_block_bytes(block);
					AES->INTFLAG.reg = AES_INTFLAG_GFMCMP;
					AES->CTRLB.reg = AES_CTRLB_GFMUL | AES_CTRLB_START;
					while (!(AES->INTFLAG.reg & AES_INTFLAG_GFMCMP)) {}
					pos += 16;
				}
			}
		}
		else
		{
			// Decrypt: GFMUL the ciphertext BEFORE decrypting, so the
			// input buffer still contains ciphertext (safe for in-place
			// where input==output). CTR-decryption happens in Phase 6
			// after tag generation, piggy-backing on the same CTR context.
			size_t pos = 0;
			while (pos < input_len)
			{
				uint8_t block[16] = {};
				const size_t n = (input_len - pos >= 16) ? 16 : (input_len - pos);
				memcpy(block, input + pos, n);
				aes_write_block_bytes(block);
				AES->INTFLAG.reg = AES_INTFLAG_GFMCMP;
				AES->CTRLB.reg = AES_CTRLB_GFMUL | AES_CTRLB_START;
				while (!(AES->INTFLAG.reg & AES_INTFLAG_GFMCMP)) {}
				pos += 16;
			}
		}

		// === Phase 5: Length Block GHASH (GFMUL) ===
		{
			uint8_t len_block[16] = {};
			const uint64_t aad_bits = (uint64_t)aad_len * 8;
			const uint64_t ct_bits  = (uint64_t)input_len * 8;
			len_block[0]  = (uint8_t)(aad_bits >> 56);
			len_block[1]  = (uint8_t)(aad_bits >> 48);
			len_block[2]  = (uint8_t)(aad_bits >> 40);
			len_block[3]  = (uint8_t)(aad_bits >> 32);
			len_block[4]  = (uint8_t)(aad_bits >> 24);
			len_block[5]  = (uint8_t)(aad_bits >> 16);
			len_block[6]  = (uint8_t)(aad_bits >> 8);
			len_block[7]  = (uint8_t)(aad_bits);
			len_block[8]  = (uint8_t)(ct_bits >> 56);
			len_block[9]  = (uint8_t)(ct_bits >> 48);
			len_block[10] = (uint8_t)(ct_bits >> 40);
			len_block[11] = (uint8_t)(ct_bits >> 32);
			len_block[12] = (uint8_t)(ct_bits >> 24);
			len_block[13] = (uint8_t)(ct_bits >> 16);
			len_block[14] = (uint8_t)(ct_bits >> 8);
			len_block[15] = (uint8_t)(ct_bits);

			aes_write_block_bytes(len_block);
			AES->INTFLAG.reg = AES_INTFLAG_GFMCMP;
			AES->CTRLB.reg = AES_CTRLB_GFMUL | AES_CTRLB_START;
			while (!(AES->INTFLAG.reg & AES_INTFLAG_GFMCMP)) {}
		}
	}

	// === Phase 6: Tag Generation (CTR mode) ===
	// Per datasheet §42.6.3.1.5
	{
		// Read current GHASH value
		uint8_t ghash[16];
		for (size_t i = 0; i < 4; i++)
		{
			uint32_t w = AES->GHASH[i].reg;
			memcpy(ghash + i * 4, &w, 4);
		}

		// Reconfigure for CTR mode (disable → set mode → enable)
		struct aes_config cfg;
		aes_get_config_defaults(&cfg);
		cfg.encrypt_mode = AES_ENCRYPTION;
		cfg.key_size     = ks;
		cfg.start_mode   = AES_MANUAL_START;
		cfg.opmode       = AES_CTR_MODE;
		aes_set_config(AES, &cfg);

		aes_write_key_bytes(key, key_len);

		// Load J0 (IV || 0x00000001) into INTVECT
		uint8_t j0[16];
		memcpy(j0, iv, 12);
		j0[12] = 0x00;
		j0[13] = 0x00;
		j0[14] = 0x00;
		j0[15] = 0x01;
		aes_write_iv_bytes(j0);

		// Load GHASH into INDATA, then NEWMSG|START
		aes_write_block_bytes(ghash);
		AES->INTFLAG.reg = AES_INTFLAG_ENCCMP;
		AES->CTRLB.reg = AES_CTRLB_NEWMSG | AES_CTRLB_START;
		aes_wait_datrdy();

		// Read tag from INDATA
		aes_read_tag_bytes(tag, tag_len);

		// For decrypt, CTR-decrypt the data now. The counter has
		// auto-incremented from 1 (used for tag) to 2, which is
		// exactly the starting counter for GCM data blocks.
		if (!encrypt && input_len > 0)
		{
			size_t pos = 0;
			while (pos + 16 <= input_len)
			{
				aes_write_block_bytes(input + pos);
				AES->INTFLAG.reg = AES_INTFLAG_ENCCMP;
				AES->CTRLB.reg = AES_CTRLB_START;
				aes_wait_datrdy();
				if (output != nullptr)
				{
					aes_read_block_bytes(output + pos);
				}
				else
				{
					uint8_t discard[16];
					aes_read_block_bytes(discard);
				}
				pos += 16;
			}
			if (pos < input_len)
			{
				uint8_t pad_in[16] = {};
				const size_t rem = input_len - pos;
				memcpy(pad_in, input + pos, rem);
				aes_write_block_bytes(pad_in);
				AES->INTFLAG.reg = AES_INTFLAG_ENCCMP;
				AES->CTRLB.reg = AES_CTRLB_START;
				aes_wait_datrdy();
				uint8_t pad_out[16];
				aes_read_block_bytes(pad_out);
				if (output != nullptr)
				{
					memcpy(output + pos, pad_out, rem);
				}
			}
		}
	}

	return 0;
}

#endif // chip selection

#endif // !AES_GCM_NOT_AVAILABLE

// ============================================================
// Public C API
// ============================================================

extern "C" int aes_gcm_decrypt_and_tag(
	const uint8_t *key, size_t key_len,
	const uint8_t *iv, size_t iv_len,
	const uint8_t *aad, size_t aad_len,
	const uint8_t *ciphertext, size_t ciphertext_len,
	uint8_t *plaintext,
	uint8_t *tag, size_t tag_len) noexcept;

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
	aes_ecb_invalidate_cache();
	return -1;
#else
	const int ret = aes_gcm_process(true, key, key_len, iv, iv_len,
							aad, aad_len, plaintext, plaintext_len,
							ciphertext, tag, tag_len);
	aes_ecb_invalidate_cache();
	return ret;
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
	aes_ecb_invalidate_cache();
	return -1;
#else
	uint8_t computed_tag[16];
	const size_t actual_tag_len = (tag_len < 16) ? tag_len : 16;

	int ret = aes_gcm_decrypt_and_tag(key, key_len, iv, iv_len,
							  aad, aad_len, ciphertext, ciphertext_len,
							  plaintext, computed_tag, actual_tag_len);

	if (ret != 0)
	{
		aes_ecb_invalidate_cache();
		return ret;
	}

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
		aes_ecb_invalidate_cache();
		return -2;
	}
	aes_ecb_invalidate_cache();
	return 0;
#endif
}

// Decrypts ciphertext to plaintext and outputs the computed authentication tag
// without performing tag comparison.  Used by the mbedTLS GCM ALT wrapper so
// that mbedtls_gcm_auth_decrypt can do its own constant-time tag comparison.
//
// Single-pass: with CIPHER=Decryption the hardware GHASHes the input side
// (ciphertext), producing the correct authentication tag directly.
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
	aes_ecb_invalidate_cache();
	return -1;
#else
	const int ret = aes_gcm_process(false, key, key_len, iv, iv_len,
							  aad, aad_len, ciphertext, ciphertext_len,
							  plaintext, tag, tag_len);
	aes_ecb_invalidate_cache();
	return ret;
#endif
}
