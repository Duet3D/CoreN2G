/*
 * SAME5x AES wrapper with ASF-compatible API surface
 */

#include <aes/aes.h>
#include <hri/hri_aes_e54.h>

static inline uint32_t aes_key_word_count(Aes *const p_aes)
{
	switch ((p_aes->CTRLA.reg & AES_CTRLA_KEYSIZE_Msk) >> AES_CTRLA_KEYSIZE_Pos)
	{
	case 0:
		return 4;
	case 1:
		return 6;
	case 2:
		return 8;
	default:
		return 4;
	}
}

void aes_get_config_defaults(struct aes_config *const p_cfg)
{
	if (p_cfg == NULL) {
		return;
	}

	p_cfg->encrypt_mode = AES_ENCRYPTION;
	p_cfg->key_size = AES_KEY_SIZE_128;
	p_cfg->start_mode = AES_MANUAL_START;
	p_cfg->opmode = AES_ECB_MODE;
	p_cfg->cfb_size = AES_CFB_SIZE_128;
	p_cfg->lod = false;
	p_cfg->gtag_en = false;
	p_cfg->processing_delay = 0;
}

void aes_init(Aes *const p_aes, struct aes_config *const p_cfg)
{
	if ((p_aes == NULL) || (p_cfg == NULL)) {
		return;
	}

	MCLK->APBCMASK.reg |= MCLK_APBCMASK_AES;
	aes_reset(p_aes);
	aes_set_config(p_aes, p_cfg);
}

void aes_enable(void)
{
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_AES;
	AES->CTRLA.reg |= AES_CTRLA_ENABLE;
}

void aes_disable(void)
{
	AES->CTRLA.reg &= ~AES_CTRLA_ENABLE;
	MCLK->APBCMASK.reg &= ~MCLK_APBCMASK_AES;
}

void aes_set_config(Aes *const p_aes, struct aes_config *const p_cfg)
{
	if ((p_aes == NULL) || (p_cfg == NULL)) {
		return;
	}

	/* Disable AES first — Enable-Protected bits (AESMODE, CIPHER, KEYSIZE,
	 * STARTMODE, LOD) can only be changed when ENABLE=0.  Writing CTRLA=0
	 * clears ENABLE and resets mode bits to defaults.  The subsequent write
	 * sets the new configuration with ENABLE=1.
	 *
	 * Note: this destroys the HASHKEY register contents.  Callers that need
	 * HASHKEY across mode switches must save and restore it explicitly, or
	 * re-write the key in GCM mode (which auto-regenerates HASHKEY). */
	hri_aes_write_CTRLA_reg(p_aes, 0);

	uint32_t ctrla = AES_CTRLA_ENABLE;

	if (p_cfg->encrypt_mode == AES_ENCRYPTION) {
		ctrla |= AES_CTRLA_CIPHER;
	}

	if (p_cfg->start_mode != AES_MANUAL_START) {
		ctrla |= AES_CTRLA_STARTMODE;
	}

	ctrla |= AES_CTRLA_KEYSIZE((uint32_t)p_cfg->key_size);
	ctrla |= AES_CTRLA_AESMODE((uint32_t)p_cfg->opmode);
	ctrla |= AES_CTRLA_CFBS((uint32_t)p_cfg->cfb_size);

	if (p_cfg->lod) {
		ctrla |= AES_CTRLA_LOD;
	}

	hri_aes_write_CTRLA_reg(p_aes, ctrla);
}

void aes_write_key(Aes *const p_aes, const uint32_t *p_key)
{
	if ((p_aes == NULL) || (p_key == NULL)) {
		return;
	}

	const uint32_t key_words = aes_key_word_count(p_aes);
	for (uint32_t i = 0; i < key_words; i++) {
		hri_aes_write_KEYWORD_reg(p_aes, (uint8_t)i, p_key[i]);
	}
}

void aes_write_initvector(Aes *const p_aes, const uint32_t *p_vector)
{
	if ((p_aes == NULL) || (p_vector == NULL)) {
		return;
	}

	for (uint8_t i = 0; i < 4; i++) {
		hri_aes_write_INTVECTV_reg(p_aes, i, p_vector[i]);
	}
}

void aes_write_input_data(Aes *const p_aes, const uint32_t *p_input_data_buffer)
{
	if ((p_aes == NULL) || (p_input_data_buffer == NULL)) {
		return;
	}

	hri_aes_write_DATABUFPTR_reg(p_aes, 0);
	for (uint8_t i = 0; i < 4; i++) {
		hri_aes_write_INDATA_reg(p_aes, p_input_data_buffer[i]);
	}
}

void aes_read_output_data(Aes *const p_aes, uint32_t *p_output_data_buffer)
{
	if ((p_aes == NULL) || (p_output_data_buffer == NULL)) {
		return;
	}

	hri_aes_write_DATABUFPTR_reg(p_aes, 0);
	for (uint8_t i = 0; i < 4; i++) {
		p_output_data_buffer[i] = hri_aes_read_INDATA_reg(p_aes);
	}
}
