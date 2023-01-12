/*
 * UniqueIdBase.h
 *
 *  Created on: 4 Oct 2021
 *      Author: David
 *
 *  This class represents a 128-bit processor unique ID as implemented on ATSAM processors.
 *  We display the unique ID as a string in the form xxxxx-xxxxx-xxxxx-xxxxx-xxxxx-xxxxx
 *  where each x is a digit or an uppercase letter that is not C, E, I or O.
 *  This encodes the original 128 bits plus 22 checksum bits.
 */

#ifndef SRC_UNIQUEIDBASE_H_
#define SRC_UNIQUEIDBASE_H_

#include <Core.h>
#include <General/StringRef.h>
#include <General/function_ref.h>

class UniqueIdBase
{
public:
	UniqueIdBase() { Clear(); }

	bool IsValid() const noexcept;
	uint32_t GetHash() const noexcept;

	void Clear() noexcept;
	void SetFromCurrentBoard() noexcept;
	void SetFromRemote(const uint8_t srcData[16]) noexcept;

	void AppendCharsTo(function_ref_noexcept<void(char) noexcept> fn) const noexcept;
	void AppendCharsToString(const StringRef& str) const noexcept;

	const uint8_t *_ecv_array GetRaw() const { return (const uint8_t *_ecv_array)data; }
	const uint32_t *_ecv_array GetDwords() const { return data; }

protected:
	void SetChecksumWord() noexcept;

#if RP2040
	uint32_t data[3];			// 64-bit unique ID plus checksum
#else
	uint32_t data[5];			// 128-bit unique ID plus checksum
#endif
};

#endif /* SRC_UNIQUEIDBASE_H_ */
