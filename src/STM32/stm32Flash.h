#ifndef FLASH_H_
#define FLASH_H_
#include <Core.h>

namespace Flash
{
bool FlashIsErased(const uint32_t addr, const size_t len) noexcept;
uint32_t FlashGetSector(const uint32_t addr) noexcept;
size_t FlashGetSectorLength(const uint32_t addr) noexcept;
bool FlashEraseSector(const uint32_t sector) noexcept;
bool FlashWrite(const uint32_t addr, const uint8_t *data, const size_t len) noexcept;
bool FlashRead(const uint32_t addr, uint8_t *data, const size_t len) noexcept;
}

#endif
