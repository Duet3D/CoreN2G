/*
 * TMC22xxUartInterface.h
 *
 *  Created on: 18 Sept 2022
 *      Author: David
 *
 * The functions in this namespace use a RP2040 PIO to drive the single-wire UART interface of one or more TMC2209 or TMC2240 drivers
 */

#ifndef SRC_RP2040_TMCUARTINTERFACE_H_
#define SRC_RP2040_TMCUARTINTERFACE_H_

#include <Core.h>

#if RP2040

#include <DmacManager.h>

namespace TmcUartInterface
{
	typedef void (*TmcUartCallbackFn)(CallbackParameter, DmaCallbackReason reason) noexcept;

	void Init(Pin uartPin, uint32_t baudRate) noexcept;
	void ResetUart() noexcept;
	void ResetDMA() noexcept;
	void SetTxData(const volatile uint32_t* data, unsigned int numWords) noexcept;
	void SetRxData(volatile uint32_t* data, unsigned int numWords) noexcept;
	void StartTransfer(TmcUartCallbackFn callbackFn) noexcept;
	void DisableCompletedCallback() noexcept;
	void AbortTransfer() noexcept;
};

#endif

#endif /* SRC_RP2040_TMCUARTINTERFACE_H_ */
