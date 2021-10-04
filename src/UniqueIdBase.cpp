/*
 * UniqueIdBase.cpp
 *
 *  Created on: 4 Oct 2021
 *      Author: David
 */

#include "UniqueIdBase.h"
#include <Cache.h>

#if SAM4E || SAM4S || SAME70
# include <Flash.h>
#endif

// Constructor initialises it to all zeros
void UniqueIdBase::Clear() noexcept
{
	for (auto& v : data)
	{
		v = 0;
	}
}

// Test if we have a valid ID. We assume a valid ID cannot be all zeros.
bool UniqueIdBase::IsValid() const noexcept
{
	return (data[0] | data[1] | data[2] | data[3]) != 0;
}

// Get a 32-bit hash of the ID. Used for pseudo random number generation.
uint32_t UniqueIdBase::GetHash() const noexcept
{
	return data[0] ^ data[1] ^ data[2] ^ data[3];
}

// Set the ID to the ID of the board that this code is running on
void UniqueIdBase::SetFromCurrentBoard() noexcept
{
#if SAME5x || SAMC21
	for (size_t i = 0; i < 4; ++i)
	{
		data[i] = *reinterpret_cast<const uint32_t*>(SerialNumberAddresses[i]);
	}
	SetChecksumWord();
#elif SAM4E || SAM4S || SAME70
	const bool cacheWasEnabled = Cache::Disable();
	const bool success = Flash::ReadUniqueId(data);
	if (cacheWasEnabled)
	{
		Cache::Enable();
	}

	if (success)
	{
		SetChecksumWord();
	}
#else
# error Unsupported processor
#endif
}

// Set the ID from 4 dwords received from a remote board
void UniqueIdBase::SetFromRemote(const uint8_t srcData[16]) noexcept
{
	memcpy(data, srcData, 16);
	SetChecksumWord();
}

// Append the unique ID in character form to something
void UniqueIdBase::AppendCharsTo(function_ref<void(char) /*noexcept*/> fn) const noexcept
{
	size_t i = 0;
	for (;;)
	{
		const size_t index = (i * 5) / 32;
		const size_t shift = (i * 5) % 32;
		uint32_t val = data[index] >> shift;
		if (shift > 32 - 5)
		{
			// We need some bits from the next dword too
			val |= data[index + 1] << (32 - shift);
		}
		val &= 31;
		char c;
		if (val < 10)
		{
			c = val + '0';
		}
		else
		{
			c = val + ('A' - 10);
			// We have 26 letters in the usual A-Z alphabet and we only need 22 of them plus 0-9.
			// So avoid using letters C, E, I and O which are easily mistaken for G, F, 1 and 0.
			if (c >= 'C')
			{
				++c;
			}
			if (c >= 'E')
			{
				++c;
			}
			if (c >= 'I')
			{
				++c;
			}
			if (c >= 'O')
			{
				++c;
			}
		}
		fn(c);

		++i;
		if (i == 30)
		{
			break;
		}

		if ((i % 5) == 0)
		{
			fn('-');
		}
	}
}

// Append the unique ID in character form to a string
void UniqueIdBase::AppendCharsToString(const StringRef &str) const noexcept
{
	AppendCharsTo([&str](char c)-> void { str.cat(c);});
}

// Set the checksum word of the unique ID
void UniqueIdBase::SetChecksumWord() noexcept
{
	// We only print 30 5-bit characters = 128 data bits + 22 checksum bits. So compress the 32 checksum bits into 22.
	data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];
	data[4] ^= (data[4] >> 10);
}

// End
