/*
 * AesEcb.cpp
 *
 * Hardware AES-ECB driver for Atmel/Microchip SAM MCUs.
 * Provides a one-shot aes_ecb_crypt() that encrypts or decrypts a single
 * 16-byte block using the on-chip AES peripheral in ECB mode.
 *
 * Supports SAME70 (using the ASF aes driver) and SAME5x (SAME54/SAME51).
 *
 * Used as the hardware back-end for the mbedTLS MBEDTLS_AES_ALT provider
 * (aes_hardware.cpp in LibMbedTls). The software GCM implementation in
 * mbedTLS drives this one block at a time, avoiding the large RAM buffers
 * required by the previous hardware GCM approach.
 *
 * Thread safety: the AES peripheral is only used from the networking task,
 * so no locking is needed. Do not call from multiple tasks simultaneously.
 */

#include <cstring>
#include <cstdint>

// ============================================================
// SAME70 AES-ECB — uses ASF aes.h
// ============================================================
#if defined(__SAME70Q20B__) || defined(__SAME70Q21B__) || defined(__SAME70N20B__) || defined(__SAME70N21B__)

#include <aes/aes.h>
#include <pmc/pmc.h>

static bool ecb_initialised = false;

/* Cached key state — key is written once per key/direction change, not per block */
static uint8_t cached_key[32];
static size_t  cached_key_len = 0;
static bool    cached_encrypt = false;
static bool    hw_configured  = false;

extern "C" void aes_ecb_invalidate_cache() noexcept
{
	hw_configured = false;
	cached_key_len = 0;
	cached_encrypt = false;
}

static void ecb_hw_init() noexcept
{
	struct aes_config init_cfg;
	aes_get_config_defaults(&init_cfg);
	aes_init(AES, &init_cfg);
	aes_enable();
}

static inline void ecb_wait_datrdy() noexcept
{
	while (!(aes_read_interrupt_status(AES) & AES_ISR_DATRDY)) {}
}

extern "C" int aes_ecb_crypt(
	bool encrypt,
	const uint8_t *key, size_t key_len,
	const uint8_t input[16], uint8_t output[16]) noexcept
{
	enum aes_key_size ks;
	switch (key_len)
	{
	case 16: ks = AES_KEY_SIZE_128; break;
	case 24: ks = AES_KEY_SIZE_192; break;
	case 32: ks = AES_KEY_SIZE_256; break;
	default: return -1;
	}

	if (!ecb_initialised)
	{
		ecb_hw_init();
		ecb_initialised = true;
		hw_configured = false;
	}

	const bool key_changed = !hw_configured
		|| key_len != cached_key_len
		|| encrypt != cached_encrypt
		|| memcmp(key, cached_key, key_len) != 0;

	if (key_changed)
	{
		struct aes_config cfg;
		aes_get_config_defaults(&cfg);
		cfg.encrypt_mode     = encrypt ? AES_ENCRYPTION : AES_DECRYPTION;
		cfg.key_size         = ks;
		cfg.start_mode       = AES_AUTO_START;
		cfg.opmode           = AES_ECB_MODE;
		cfg.lod              = false;
		cfg.gtag_en          = false;
		cfg.processing_delay = 0;
		aes_set_config(AES, &cfg);

		uint32_t key_words[8] = {};
		memcpy(key_words, key, key_len);
		aes_write_key(AES, key_words);

		memcpy(cached_key, key, key_len);
		cached_key_len = key_len;
		cached_encrypt = encrypt;
		hw_configured  = true;
	}

	uint32_t in_words[4];
	memcpy(in_words, input, 16);
	aes_write_input_data(AES, in_words);     // AUTO_START triggers on last word write
	ecb_wait_datrdy();

	uint32_t out_words[4];
	aes_read_output_data(AES, out_words);
	memcpy(output, out_words, 16);

	return 0;
}

// ============================================================
// SAME5x (SAME54 / SAME51) AES-ECB — uses ASF aes.h
//
// Mirrors the SAME70 driver: init once, auto-start, cache key.
// ============================================================
#elif defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAME51P20A__) || defined(__SAME54N20A__)

#include <aes/aes.h>

static bool ecb_initialised = false;

/* Cached key state — key is written once per key/direction change, not per block */
static uint8_t cached_key[32];
static size_t  cached_key_len = 0;
static bool    cached_encrypt = false;
static bool    hw_configured  = false;

extern "C" void aes_ecb_invalidate_cache() noexcept
{
	hw_configured = false;
	cached_key_len = 0;
	cached_encrypt = false;
}

static void ecb_hw_init() noexcept
{
	struct aes_config init_cfg;
	aes_get_config_defaults(&init_cfg);
	aes_init(AES, &init_cfg);
	aes_enable();
}

static inline void ecb_wait_datrdy() noexcept
{
	while (!(aes_read_interrupt_status(AES) & AES_ISR_DATRDY)) {}
}

extern "C" int aes_ecb_crypt(
	bool encrypt,
	const uint8_t *key, size_t key_len,
	const uint8_t input[16], uint8_t output[16]) noexcept
{
	enum aes_key_size ks;
	switch (key_len)
	{
	case 16: ks = AES_KEY_SIZE_128; break;
	case 24: ks = AES_KEY_SIZE_192; break;
	case 32: ks = AES_KEY_SIZE_256; break;
	default: return -1;
	}

	if (!ecb_initialised)
	{
		ecb_hw_init();
		ecb_initialised = true;
		hw_configured = false;
	}

	const bool key_changed = !hw_configured
		|| key_len != cached_key_len
		|| encrypt != cached_encrypt
		|| memcmp(key, cached_key, key_len) != 0;

	if (key_changed)
	{
		struct aes_config cfg;
		aes_get_config_defaults(&cfg);
		cfg.encrypt_mode     = encrypt ? AES_ENCRYPTION : AES_DECRYPTION;
		cfg.key_size         = ks;
		cfg.start_mode       = AES_AUTO_START;
		cfg.opmode           = AES_ECB_MODE;
		cfg.lod              = false;
		cfg.gtag_en          = false;
		cfg.processing_delay = 0;
		aes_set_config(AES, &cfg);

		uint32_t key_words[8] = {};
		memcpy(key_words, key, key_len);
		aes_write_key(AES, key_words);

		memcpy(cached_key, key, key_len);
		cached_key_len = key_len;
		cached_encrypt = encrypt;
		hw_configured  = true;
	}

	uint32_t in_words[4];
	memcpy(in_words, input, 16);
	aes_write_input_data(AES, in_words);     // AUTO_START triggers on last word write
	ecb_wait_datrdy();

	uint32_t out_words[4];
	aes_read_output_data(AES, out_words);
	memcpy(output, out_words, 16);

	return 0;
}

// ============================================================
// Fallback for unsupported targets
// ============================================================
#else

extern "C" int aes_ecb_crypt(
	bool /*encrypt*/,
	const uint8_t * /*key*/, size_t /*key_len*/,
	const uint8_t /*input*/[16], uint8_t /*output*/[16]) noexcept
{
	return -1;
}

extern "C" void aes_ecb_invalidate_cache() noexcept
{
}

#endif
