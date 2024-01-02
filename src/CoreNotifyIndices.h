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
	constexpr uint32_t UartRx = NextAvailableAfterRTOS;
	constexpr uint32_t UartTx = UartRx;						// we can use the same one because a task can;t be waiting for both Rx and Tx
	constexpr uint32_t CanDevice = UartRx;					// CAN can share with UART
	constexpr uint32_t AnalogIn = UartRx + 1;				// analog in needs its own because it calls a hook function
	constexpr uint32_t NextAvailableAfterCore = NextAvailableAfterRTOS + 2;
}

#endif /* SRC_CORENOTIFYINDICES_H_ */
