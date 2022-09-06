/*
 * VirtualCanRegisters.h
 *
 *  Created on: 26 Aug 2022
 *      Author: David
 */

#ifndef SRC_RP2040_VIRTUALCANREGISTERS_H_
#define SRC_RP2040_VIRTUALCANREGISTERS_H_

#include <CoreIO.h>
#include <CanFilterElements.h>

/**@}*/
/**
 * \brief CAN receive FIFO element.
 */
struct CanRxBufferHeader
{
	union
	{
		struct
		{
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R0;
	union
	{
		struct
		{
			uint32_t RXTS : 16; /*!< Rx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t : 2;       /*!< Reserved */
			uint32_t FIDX : 7;  /*!< Filter Index */
			uint32_t ANMF : 1;  /*!< Accepted Non-matching Frame */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;

	const volatile uint32_t *GetDataPointer() const volatile { return (volatile uint32_t*)this + (sizeof(*this)/sizeof(uint32_t)); }
};

struct CanRxBuffer : public CanRxBufferHeader
{
	uint32_t data[64/4];
};

/**
 * \brief CAN transmit FIFO element.
 */
struct CanTxBufferHeader
{
	union
	{
		struct
		{
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T0;
	union
	{
		struct
		{
			uint32_t : 16;    /*!< Reserved */
			uint32_t DLC : 4; /*!< Data Length Code */
			uint32_t BRS : 1; /*!< Bit Rate Switch */
			uint32_t FDF : 1; /*!< FD Format */
			uint32_t : 1;     /*!< Reserved */
			uint32_t EFCbit : 1; /*!< Event FIFO Control */
			uint32_t MM : 8;  /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T1;

	volatile uint32_t *GetDataPointer() volatile { return (volatile uint32_t*)this + (sizeof(*this)/sizeof(uint32_t)); }
};

struct CanTxBuffer : public CanTxBufferHeader
{
	union
	{
		uint8_t data8[64];
		uint16_t data16[64/2];
		uint32_t data32[64/4];
	};
};

static constexpr size_t NumCanRxFifos = 2;

struct CanErrorCounts
{
	uint32_t stuffCountParity;
	uint32_t wrongStuffCount;
	uint32_t wrongCrc;
	uint32_t wrongMessageType;
	uint32_t missingCrcDelimiter;
	uint32_t noAck;
	uint32_t missingEofBit1;
	uint32_t missingEofBit2;
	uint32_t badStuffing;
	uint32_t tooLateToAck;
	uint32_t rxFifoOverlow[NumCanRxFifos];

	// Clear all error counts
	void Clear() noexcept
	{
		stuffCountParity = wrongStuffCount = wrongCrc = wrongMessageType = missingCrcDelimiter
			= noAck = missingEofBit1 = missingEofBit2 = badStuffing = tooLateToAck = 0;
		for (volatile uint32_t& err : rxFifoOverlow)
		{
			err = 0;
		}
	}
};

struct VirtualCanRegisters
{
	struct RxFifo
	{
		uint32_t size;									// written be proc 0, during setup only
		volatile CanRxBuffer *volatile buffers;			// written be proc 0, during setup only
		volatile uint32_t getIndex;						// only written by proc 0
		volatile uint32_t putIndex;						// initialised by proc0 then only written by CAN

		void Clear() noexcept { getIndex = 0; putIndex = 0; }
	};

	struct TxFifo
	{
		uint32_t size;									// written be proc 0, during setup only
		volatile CanTxBuffer *volatile buffers;			// written be proc 0, during setup only
		volatile uint32_t getIndex;						// initialised by proc0 then only written by CAN
		volatile uint32_t putIndex;						// only written by proc 0

		void Clear() noexcept { getIndex = 0; putIndex = 0; }
	};

	RxFifo rxFifos[NumCanRxFifos];
	TxFifo txFifo;

	// The following configuration registers are written by the main processor while CAN is disabled, and never changed while CAN is enabled
	volatile unsigned int numShortFilterElements;
	volatile unsigned int numExtendedFilterElements;
	volatile CanStandardMessageFilterElement *volatile shortFiltersAddr;	// start address of short filter elements
	volatile CanExtendedMessageFilterElement *volatile extendedFiltersAddr;	// start address of extended filter elements

	// The following are written only by CAN
	CanErrorCounts errors;

	// The following are written only by the main processor
	volatile bool txFifoNotFullInterruptEnabled;
	volatile bool cancelTransmission;
	volatile bool canEnabled;										// CAN enable/disable flag, set by main proc to enable CAN after writing other registers, cleared by main proc to disable CAN
	volatile bool clearErrorCounts;

	// The following are written by the main processor prior to initial setup
	uint32_t bitrate;
	Pin txPin;
	Pin rxPin;

	// Bit assignments in the pseudo-interrupt message received by the main processor via the inter-processor fifo from the CAN processor
	static constexpr uint32_t recdFifo0 = 0x01;						// message received in fifo0
	static constexpr uint32_t recdFifo1 = 0x02;						// message received in fifo1
	static constexpr uint32_t txFifoNotFull = 0x08;					// space has been made in the transmit fifo

	static_assert(recdFifo1 == recdFifo0 << 1);						// the code assumes we can shift recdFifo0 left by the fifo number

	// Disable CAN and initialise the virtual registers
	void Init() noexcept
	{
		canEnabled = false;
		cancelTransmission = txFifoNotFullInterruptEnabled = clearErrorCounts = false;
		for (RxFifo& fifo : rxFifos)
		{
			fifo.Clear();
		}
		txFifo.Clear();
		errors.Clear();
	}
};

#endif /* SRC_RP2040_VIRTUALCANREGISTERS_H_ */
