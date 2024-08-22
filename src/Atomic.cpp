/*
 * Atomic.cpp
 *
 *  Created on: 9 Mar 2021
 *      Author: David
 */

// Atomic functions for ARM Cortex M0/M0+ processors missing from the standard library
// There are many other such functions missing. They can be added if/when the linker complains about not finding them.
// See section 6.55 of the gcc manual for more information about them.
// CAUTION: this version does not handle the case of both cores of the RP2040 accessing the same atomic variable!

#include <Core.h>

#if SAMC21 || RP2040

extern "C" uint8_t __atomic_sub_fetch_1(volatile void *ptr, uint8_t val, int memorder) noexcept
{
	const irqflags_t flags = IrqSave();
	uint8_t ret = *(volatile uint8_t*)ptr;
	ret -= val;
	*(volatile uint8_t*)ptr = ret;
	IrqRestore(flags);
	return ret;
}

extern "C" uint8_t __atomic_fetch_sub_1(volatile void *ptr, uint8_t val, int memorder) noexcept
{
	const irqflags_t flags = IrqSave();
	const uint8_t ret = *(volatile uint8_t*)ptr;
	*(volatile uint8_t*)ptr = ret - val;
	IrqRestore(flags);
	return ret;
}

extern "C" bool __atomic_compare_exchange_1(volatile void *ptr, void *expected, uint8_t desired, bool weak, int success_memorder, int failure_memorder) noexcept
{
	bool ret;
	const irqflags_t flags = IrqSave();
	const uint8_t actual = *(volatile uint8_t*)ptr;
	if (*(uint8_t*)expected == actual)
	{
		*(volatile uint8_t*)ptr = desired;
		ret = true;
	}
	else
	{
		*(uint8_t*)expected = actual;
		ret = false;
	}
	IrqRestore(flags);
	return ret;
}

extern "C" unsigned int __atomic_fetch_or_4(volatile void *ptr, unsigned int val, int memorder) noexcept
{
	const irqflags_t flags = IrqSave();
	const unsigned int ret = *(volatile unsigned int*)ptr;
	*(volatile unsigned int*)ptr = ret | val;
	IrqRestore(flags);
	return ret;
}

extern "C" unsigned int __atomic_fetch_and_4(volatile void *ptr, unsigned int val, int memorder) noexcept
{
	const irqflags_t flags = IrqSave();
	const unsigned int ret = *(volatile unsigned int*)ptr;
	*(volatile unsigned int*)ptr = ret & val;
	IrqRestore(flags);
	return ret;
}

extern "C" unsigned int __atomic_fetch_add_4(volatile void *ptr, unsigned int val, int memorder) noexcept
{
	const irqflags_t flags = IrqSave();
	const unsigned int ret = *(volatile unsigned int*)ptr;
	*(volatile unsigned int*)ptr = ret + val;
	IrqRestore(flags);
	return ret;
}

#endif

// End
