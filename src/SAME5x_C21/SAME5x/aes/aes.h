/*
 * SAME5x AES wrapper with ASF-compatible API surface
 */

#ifndef AES_H_INCLUDED
#define AES_H_INCLUDED

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

enum aes_encrypt_mode {
	AES_DECRYPTION = 0,
	AES_ENCRYPTION,
};

enum aes_key_size {
	AES_KEY_SIZE_128 = 0,
	AES_KEY_SIZE_192,
	AES_KEY_SIZE_256,
};

enum aes_start_mode {
	AES_MANUAL_START = 0,
	AES_AUTO_START,
	AES_IDATAR0_START,
};

enum aes_opmode {
	AES_ECB_MODE = 0,
	AES_CBC_MODE,
	AES_OFB_MODE,
	AES_CFB_MODE,
	AES_CTR_MODE,
	AES_CCM_MODE,
	AES_GCM_MODE,
};

enum aes_cfb_size {
	AES_CFB_SIZE_128 = 0,
	AES_CFB_SIZE_64,
	AES_CFB_SIZE_32,
	AES_CFB_SIZE_16,
	AES_CFB_SIZE_8,
};

struct aes_config {
	enum aes_encrypt_mode encrypt_mode;
	enum aes_key_size key_size;
	enum aes_start_mode start_mode;
	enum aes_opmode opmode;
	enum aes_cfb_size cfb_size;
	bool lod;
	bool gtag_en;
	uint32_t processing_delay;
};

void aes_get_config_defaults(struct aes_config *const p_cfg);
void aes_init(Aes *const p_aes, struct aes_config *const p_cfg);
void aes_enable(void);
void aes_disable(void);
void aes_set_config(Aes *const p_aes, struct aes_config *const p_cfg);
void aes_write_key(Aes *const p_aes, const uint32_t *p_key);
void aes_write_initvector(Aes *const p_aes, const uint32_t *p_vector);
void aes_write_input_data(Aes *const p_aes, const uint32_t *p_input_data_buffer);
void aes_read_output_data(Aes *const p_aes, uint32_t *p_output_data_buffer);

static inline void aes_reset(Aes *const p_aes)
{
	(void)p_aes;
	AES->CTRLA.reg |= AES_CTRLA_SWRST;
	while ((AES->CTRLA.reg & AES_CTRLA_SWRST) != 0) {
	}
}

static inline void aes_start(Aes *const p_aes)
{
	p_aes->CTRLB.reg = AES_CTRLB_START;
}

static inline uint32_t aes_read_interrupt_status(Aes *const p_aes)
{
	return p_aes->INTFLAG.reg;
}

static inline uint32_t aes_read_interrupt_mask(Aes *const p_aes)
{
	return p_aes->INTENSET.reg;
}

static inline void aes_write_authen_datalength(Aes *const p_aes, uint32_t length)
{
	(void)p_aes;
	(void)length;
}

static inline void aes_write_pctext_length(Aes *const p_aes, uint32_t length)
{
	p_aes->CIPLEN.reg = length;
}

static inline uint32_t aes_read_tag(Aes *const p_aes, uint32_t id)
{
	return p_aes->GHASH[id & 0x03u].reg;
}

/* SAME70 code waits on AES_ISR_DATRDY; on SAME5x completion uses ENCCMP. */
#define AES_ISR_DATRDY AES_INTFLAG_ENCCMP
#define AES_ISR_TAGRDY AES_INTFLAG_GFMCMP

#ifdef __cplusplus
}
#endif

#endif
