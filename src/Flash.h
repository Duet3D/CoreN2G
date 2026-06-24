/*
 * Flash.h
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_

#include <CoreIO.h>

namespace Flash
{
#if STM32
	bool FlashIsErased(const uint32_t addr, const size_t len) noexcept;
	uint32_t FlashGetSector(const uint32_t addr) noexcept;
	size_t FlashGetSectorLength(const uint32_t addr) noexcept;
	bool FlashEraseSector(const uint32_t sector) noexcept;
	bool FlashWrite(const uint32_t addr, const uint8_t *data, const size_t len) noexcept;
	bool FlashRead(const uint32_t addr, uint8_t *data, const size_t len) noexcept;
#else
	bool Init() noexcept;
	void Deinit() noexcept;
	bool Unlock(uint32_t start, uint32_t length) noexcept;
	bool Lock(uint32_t start, uint32_t length) noexcept;
	bool Write(uint32_t start, uint32_t length, const uint32_t *_ecv_array data) noexcept;
# if SAM4S || SAM4E || SAME70
	bool EraseSector(uint32_t) noexcept;
# else
	bool Erase(uint32_t start, uint32_t length) noexcept;
# endif

# if SAMC21
	bool RwwErase(uint32_t start, uint32_t length) noexcept;
	bool RwwWrite(uint32_t start, uint32_t length, const uint8_t *data) noexcept;
 #elif SAM4S || SAM4E || SAME70
	bool ReadUserSignature(uint32_t *_ecv_array p_data, uint32_t ul_size) noexcept;
	bool WriteUserSignature(const uint32_t *_ecv_array p_buffer) noexcept;
	bool EraseUserSignature() noexcept;
	bool ReadUniqueId(uint32_t *_ecv_array pul_data) noexcept;
	int IsGpNvmSet(uint32_t gpnvm) noexcept;
	uint32_t ReadGpNvmBits() noexcept;
	bool ClearGpNvm(uint32_t gpnvm) noexcept;
# endif

	uint32_t GetLastFlashError() noexcept;
	uint32_t GetPageSize() noexcept;
	uint32_t GetLockRegionSize() noexcept;
	uint32_t GetEraseRegionSize() noexcept;
	uint32_t GetFlashSize() noexcept;
#endif
}

extern "C" uint32_t GetFlashSize_C() noexcept;

#endif /* SRC_FLASH_H_ */
