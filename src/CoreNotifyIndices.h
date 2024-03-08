/*
 * CoreNotifyIndices.h
 *
 *  Created on: 2 Jan 2024
 *      Author: David
 *
 *  Definitions of task notification indices used by the RTOS interface layer
 */

#ifndef SRC_CORENOTIFYINDICES_H_
#define SRC_CORENOTIFYINDICES_H_

#include <RTOSIface/RTOSNotifyIndices.h>

namespace NotifyIndices
{
	constexpr uint32_t UartTx = NextAvailableAfterRTOS;
	constexpr uint32_t CanDevice = UartTx;					// CAN can share with UART
	constexpr uint32_t Usb = UartTx;
	constexpr uint32_t Sdhc = UartTx;
	constexpr uint32_t AnalogIn = UartTx + 1;				// analog in best have its own because it calls a hook function that may call device drivers
	constexpr uint32_t NextAvailableAfterCore = NextAvailableAfterRTOS + 2;
}

#endif /* SRC_CORENOTIFYINDICES_H_ */
