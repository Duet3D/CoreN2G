// NVRAM emulation for STM32H5 and STM32H7 processors

#include "NVMEmulation.h"

#if STM32

#include <CoreIO.h>
#include <Cache.h>
#include <Flash.h>

// On STM32 we store reset data in flash. We have allocated a flash sector for each page for this purpose and these arrays maps to them.
__attribute__((__section__(".reset_data"), used)) uint32_t ResetData[FLASH_DATA_LENGTH/sizeof(uint32_t)];
__attribute__((__section__(".closed_loop_data"), used)) uint32_t ClosedLoopData[FLASH_DATA_LENGTH/sizeof(uint32_t)];

constexpr uint32_t SlotSize = 512/sizeof(uint32_t);					// in 32 bit words
constexpr uint32_t MAX_SLOT = (FLASH_DATA_LENGTH/sizeof(uint32_t))/SlotSize - 1;

// This code assumes that only one task or exception handler will try to access NVM at a time, so we can use static variables here.
static uint32_t currentSlot = MAX_SLOT+1;
static bool slotIsErased = false;

static uint32_t *GetPagePtr(uint8_t page) noexcept
{
	return (page == 0) ? ResetData : ClosedLoopData;
}

static uint32_t *GetSlotPtr(uint8_t page, uint8_t slot) noexcept
{
    return GetPagePtr(page) + (slot * SlotSize);
}

// When the Sector is erased, all the bits will be high
// The first 2 bytes of a used slot will have the magic number in it.
bool IsSlotVacant(uint8_t page, uint8_t slot) noexcept
{
    return Flash::IsErased(reinterpret_cast<uint32_t>(GetSlotPtr(page, slot)), SlotSize);
}

void NVMEmulationRead(uint8_t page, void *data, uint32_t dataLength) noexcept
{
    // Find the most recently written data or slot 0 if all free
    currentSlot = MAX_SLOT;
    while (currentSlot > 0 && IsSlotVacant(page, currentSlot)) { currentSlot--; }
    slotIsErased = (currentSlot == 0 && IsSlotVacant(page, currentSlot));
    uint32_t *const slotStartAddress = GetSlotPtr(page, currentSlot);
    Flash::Read(reinterpret_cast<uint32_t>(slotStartAddress), dataLength, (uint8_t *)data);
}

bool NVMEmulationErase(uint8_t page) noexcept
{
	if (slotIsErased) { return true; }

    if (currentSlot < MAX_SLOT)				    // have we reached the last slot yet?
    {
        currentSlot++;
    }
    else
    {
		Flash::EraseSectors(Flash::GetSector(reinterpret_cast<uint32_t>(GetPagePtr(page))), 1);
		currentSlot = 0;
    }
    slotIsErased = true;
    return true;
}

bool NVMEmulationWrite(uint8_t page, const void *data, uint32_t dataLength) noexcept
{
    if (dataLength != SlotSize * sizeof(uint32_t))
    {
        return false;
    }

    if (!slotIsErased)
    {
		if (++currentSlot > MAX_SLOT)
		{
			// All slots have been used, erase entire sector
			Flash::EraseSectors(Flash::GetSector(reinterpret_cast<uint32_t>(GetPagePtr(page))), 1);
			if (!Flash::IsErased(reinterpret_cast<uint32_t>(GetPagePtr(page)), sizeof(ResetData)))
			{
				return false;
			}
			currentSlot = 0;
		}
    }

    return Flash::Write(reinterpret_cast<uint32_t>(GetSlotPtr(page, currentSlot)), dataLength, (const uint32_t*)data);
}

#endif

// End
