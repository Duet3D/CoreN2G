/*
 * tusb_config.h
 *
 *  Created on: 29 Aug 2022
 *      Author: David
 */

#ifndef SRC_RP2040_TUSB_CONFIG_H_
#define SRC_RP2040_TUSB_CONFIG_H_

#define CFG_TUSB_MCU				OPT_MCU_RP2040
#define CFG_TUSB_OS					OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE		(OPT_MODE_DEVICE | OPT_MODE_LOW_SPEED)
#define CFG_TUD_CDC					1

#endif /* SRC_RP2040_TUSB_CONFIG_H_ */
