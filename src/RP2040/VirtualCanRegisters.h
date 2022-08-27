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

struct VirtualCanRegisters
{
	// The following configuration registers are written by the main processor while CAN is disabled, and never changed while CAN is enabled
	unsigned int rxFifo0Size;										// number of entries in fifo 0
	unsigned int rxFifo1Size;										// number of entries in fifo 1
	unsigned int txFifoSize;										// number of entries in transmit fifo
	volatile void *rxFifo0Addr;										// fifo 0 start address
	volatile void *rxFifo1Addr;										// fifo 1 start address
	volatile void *rxBuffersAddr;									// dedicated receive buffers start address
	volatile void *txFifoAddr;										// transmit fifo address
	unsigned int numShortFilterElements;
	unsigned int numExtendedFilterElements;
	CanStandardMessageFilterElement *shortFiltersAddr;				// start address of short filter elements
	CanExtendedMessageFilterElement *extendedFiltersAddr;			// start address of short filter elements

	// The following are written only by CAN
	volatile uint32_t rxBuffersReceivedMap;							// bitmap of dedicated receive buffers containing received messages
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

	// Bit assignments in the pseudo-interrupt message received by the main processor via the inter-processor fifo from the CAN processor
	static constexpr uint32_t recdFifo0 = 0x01;						// message received in fifo0
	static constexpr uint32_t recdFifo1 = 0x02;						// message received in fifo0
	static constexpr uint32_t recdBuff = 0x04;						// message received in dedicated buffer
	static constexpr uint32_t txDone = 0x08;						// transmission complete
	static constexpr uint32_t rxOvfFifo0 = 0x10;					// lost message destined for fifo0 because fifo was full
	static constexpr uint32_t rxOvfFifo1 = 0x20;					// lost message destined for fifo1 because fifo was full

	void Init() noexcept
	{
		canEnabled = false;
		cancelTransmission = txFifoNotFullInterruptEnabled = false;
		rxBuffersReceivedMap = 0;
		rxFifo0PutIndex = rxFifo1PutIndex = txFifoPutIndex = 0;
		rxFifo0GetIndex = rxFifo1GetIndex = txFifoGetIndex = 0;
	}

};

#endif /* SRC_RP2040_VIRTUALCANREGISTERS_H_ */
