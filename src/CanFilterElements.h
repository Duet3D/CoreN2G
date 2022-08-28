/*
 * CanFilterElements.h
 *
 *  Created on: 27 Aug 2022
 *      Author: David
 */

#ifndef SRC_CANFILTERELEMENTS_H_
#define SRC_CANFILTERELEMENTS_H_

#include <Core.h>

/**
 * \brief CAN standard message ID filter element structure.
 *
 *  Common element structure for standard message ID filter element.
 */
struct CanStandardMessageFilterElement
{
#if RP2040
	uint32_t id: 11,
			 whichBuffer : 3,
			 mask : 11,
			 enabled : 1;

	bool Matches(uint32_t msgId) const noexcept
	{
		return enabled && (msgId & mask) == id;
	}
#else
	union S0Type
	{
		struct
		{
			uint32_t SFID2 : 11; /*!< Standard Filter ID 2 */
			uint32_t : 5;        /*!< Reserved */
			uint32_t SFID1 : 11; /*!< Standard Filter ID 1 */
			uint32_t SFEC : 3;   /*!< Standard Filter Configuration */
			uint32_t SFT : 2;    /*!< Standard Filter Type */
		} bit;
		uint32_t val; /*!< Type used for register access */
	};

	__IO S0Type S0;
#endif
};

/**
 * \brief CAN extended message ID filter element structure.
 *
 *  Common element structure for extended message ID filter element.
 */
struct CanExtendedMessageFilterElement
{
#if RP2040
	uint32_t id : 29,
			 whichBuffer : 3,
			 mask : 29,
			 enabled : 1;

	bool Matches(uint32_t msgId) const noexcept
	{
		return enabled && (msgId & mask) == id;
	}
#else
	union F0Type
	{
		struct
		{
			uint32_t EFID1 : 29;	//!< bit: Extended Filter ID 1
			uint32_t EFEC : 3;		//!< bit: Extended Filter Configuration
		} bit;
		uint32_t val;				//!< Type used for register access
	};

	union F1Type
	{
		struct
		{
			uint32_t EFID2 : 29;	//!< bit: Extended Filter ID 2
			uint32_t : 1;			//!< bit: Reserved
			uint32_t EFT : 2;		//!< bit: Extended Filter Type
		} bit;
		uint32_t val;				//!< Type used for register access
	};

	__IO union F0Type F0;
	__IO union F1Type F1;
#endif
};

#endif /* SRC_CANFILTERELEMENTS_H_ */
