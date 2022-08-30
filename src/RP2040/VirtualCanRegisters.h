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

struct VirtualCanRegisters
{
	// The following configuration registers are written by the main processor while CAN is disabled, and never changed while CAN is enabled
	volatile unsigned int rxFifo0Size;										// number of entries in fifo 0
	volatile unsigned int rxFifo1Size;										// number of entries in fifo 1
	volatile unsigned int txFifoSize;										// number of entries in transmit fifo
	volatile CanRxBuffer *volatile rxFifo0Addr;								// fifo 0 start address
	volatile CanRxBuffer *volatile rxFifo1Addr;								// fifo 1 start address
	volatile CanTxBuffer *volatile txFifoAddr;								// transmit fifo address
	volatile unsigned int numShortFilterElements;
	volatile unsigned int numExtendedFilterElements;
	volatile CanStandardMessageFilterElement *volatile shortFiltersAddr;	// start address of short filter elements
	volatile CanExtendedMessageFilterElement *volatile extendedFiltersAddr;	// start address of short filter elements

	// The following are written only by CAN
	volatile unsigned int rxFifo0PutIndex;
	volatile unsigned int rxFifo1PutIndex;
	volatile unsigned int txFifoGetIndex;
	volatile uint32_t regError;										// error register. CAN or's bits into it, main proc clears it

	// The following are written only by the main processor
	volatile unsigned int rxFifo0GetIndex;
	volatile unsigned int rxFifo1GetIndex;
	volatile unsigned int txFifoPutIndex;
	volatile bool txFifoNotFullInterruptEnabled;
	volatile bool cancelTransmission;
	volatile bool canEnabled;										// CAN enable/disable flag, set by main proc to enable CAN after writing other registers, cleared by main proc to disable CAN

	// The following are written by the main processor prior to initial setup
	uint32_t bitrate;
	Pin txPin;
	Pin rxPin;
	uint8_t pioNumber;

	// Bit assignments in the pseudo-interrupt message received by the main processor via the inter-processor fifo from the CAN processor
	static constexpr uint32_t recdFifo0 = 0x01;						// message received in fifo0
	static constexpr uint32_t recdFifo1 = 0x02;						// message received in fifo0
	static constexpr uint32_t txDone = 0x08;						// transmission complete
	static constexpr uint32_t rxOvfFifo0 = 0x10;					// lost message destined for fifo0 because fifo was full
	static constexpr uint32_t rxOvfFifo1 = 0x20;					// lost message destined for fifo1 because fifo was full

	// Disable CAN and initialise the virtual registers
	void Init() noexcept
	{
		canEnabled = false;
		cancelTransmission = txFifoNotFullInterruptEnabled = false;
		rxFifo0PutIndex = rxFifo1PutIndex = txFifoPutIndex = 0;
		rxFifo0GetIndex = rxFifo1GetIndex = txFifoGetIndex = 0;
	}
};

#endif /* SRC_RP2040_VIRTUALCANREGISTERS_H_ */
