/**
 * \file
 *
 * \brief SAM USBHS HPL
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef _HPL_USBHS_HOST_H_INCLUDED
#define _HPL_USBHS_HOST_H_INCLUDED

#include "hpl_usb_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief      Private USB host controller driver data structure
 */
struct _usb_h_prvt {
	/** Pointer to pipe pool start address */
	struct usb_h_pipe *pipe_pool;
	/** Pipes to unfreeze after wakeup */
	uint16_t pipes_unfreeze;
	/** DPRAM used bytes */
	uint16_t dpram_used;
	/** Delayed suspend time in ms */
	int8_t suspend_start;
	/** Delayed resume time in ms */
	int8_t resume_start;
	/** Control transfer request user count */
	int8_t n_ctrl_req_user;
	/** SOF user count (callback, suspend, resume, ctrl request) */
	int8_t n_sof_user;
	/** Pipe pool size in number of pipes */
	uint8_t pipe_pool_size;
};

/**
 * @brief      Initialize the private data for Host Controller Driver
 *
 * @param[out] prvt            Pointer to the private data instance
 * @param[in]  pipe_pool       The pipe pool
 * @param[in]  pipe_pool_size  The pipe pool size
 */
static inline void _usbhcd_prvt_init(struct _usb_h_prvt *prvt, struct usb_h_pipe *pipe_pool, uint8_t pipe_pool_size)
{
	prvt->pipe_pool      = pipe_pool;
	prvt->pipe_pool_size = pipe_pool_size;
}

#ifdef __cplusplus
}
#endif

#endif /* _HPL_USBHS_HOST_H_INCLUDED */
