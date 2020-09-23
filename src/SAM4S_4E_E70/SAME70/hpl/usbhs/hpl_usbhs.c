/**
 * \file
 *
 * \brief SAM USBHS HPL
 *
 * Copyright (c) 2016-2019 Microchip Technology Inc. and its subsidiaries.
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

#include <compiler.h>
#include <hal_atomic.h>
#include <string.h>
#include <hpl_usb_device.h>

#include <utils_assert.h>
#include <peripheral_clk_config.h>

#undef DEPRECATED
#define DEPRECATED(macro, message)

#ifndef USB_SPEED_HIGH_SPEED
#define USB_SPEED_HIGH_SPEED 2
#endif

#ifndef USBHS_RAM_ADDR
#define USBHS_RAM_ADDR 0xA0100000u
#endif

/** Endpoint/pipe FIFO access range */
#define USBHS_RAM_EP_SIZE 0x8000

/** Total DPRAM for endpoint/pipe to allocate */
#define USBHS_DPRAM_SIZE 4096

/** Max DMA transfer size */
#define USBHS_DMA_TRANS_MAX 0x10000

/** Check if the address is 32-bit aligned. */
#define _usb_is_aligned(val) (((uint32_t)(val)&0x3) == 0)

#if (CONF_USBHS_SRC == CONF_SRC_USB_48M)
#if (CONF_USB_D_SPEED == USB_SPEED_HIGH_SPEED)
#error In low-power mode, only full speed and low speed are available.
#endif
#endif

/** Check if the buffer supports USB DMA. */
#define _usbhs_buf_dma_sp(buf) (1) /* SRAM/Flash/EBI all supports DMA */

/** Timeout between control data packets : 500ms */
#define USB_CTRL_DPKT_TIMEOUT (500)
/** Timeout of status packet : 50ms */
#define USB_CTRL_STAT_TIMEOUT (50)

/** Get 64-, 32-, 16- or 8-bit access to FIFO data register of selected endpoint.
 * \param[in] epn   Endpoint of which to access FIFO data register
 * \param[in] scale Data scale in bits: 64, 32, 16 or 8
 * \return Volatile 64-, 32-, 16- or 8-bit data pointer to FIFO data register
 * \note It is up to the user of this macro to make sure that all accesses
 *       are aligned with their natural boundaries except 64-bit accesses which
 *       require only 32-bit alignment.
 *       It is up to the user of this macro to make sure that used HSB
 *       addresses are identical to the DPRAM internal pointer modulo 32 bits.
 */
#define _usbhs_get_pep_fifo_access(epn, scale)                                                                         \
	(((volatile uint##scale##_t(*)[USBHS_RAM_EP_SIZE / ((scale) / 8)]) USBHS_RAM_ADDR)[(epn)])

/**
 * \brief Dummy callback function
 */
static void _dummy_func_no_return(uint32_t unused0, uint32_t unused1)
{
	(void)unused0;
	(void)unused1;
}

/**
 * \brief Dummy callback function
 * \return Always false.
 */
static bool _dummy_func_return_false(uint32_t unused0, uint32_t unused1)
{
	(void)unused0;
	(void)unused1;
	return false;
}

/** Retry for USB remote wakeup sending. */
#define CONF_USB_RMT_WKUP_RETRY 5

/** \name HW specific settings and implements */
/*@{*/

/** Number of endpoints supported. */
#define USB_D_N_EP CONF_USB_D_NUM_EP_SP

/** HPL USB device endpoint struct. */
struct _usb_d_dev_ep {
	/** Pointer to transaction buffer. */
	uint8_t *trans_buf;
	/** Transaction size. */
	uint32_t trans_size;
	/** Transaction transferred count. */
	uint32_t trans_count;
	/** Size of data loaded (or prepared for DMA) last time. */
	uint32_t trans_load;

	/** Endpoint size. */
	uint16_t size;
	/** Endpoint address. */
	uint8_t ep;
	/** Feature flags. */
	union {
		/** Interpreted by bit fields. */
		struct {
			/** EPCFG.ETYPE. */
			uint8_t eptype : 3;
			/** Stall status. */
			uint8_t is_stalled : 1;
			/** Transaction auto ZLP. */
			uint8_t need_zlp : 1;
			/** Reserve. */
			uint8_t : 1;
			/** Endpoint is busy. */
			uint8_t is_busy : 1;
			/** Transaction direction. */
			uint8_t dir : 1;
		} bits;
		uint8_t u8;
	} flags;
};

/** Check if the endpoint is used. */
#define _usb_d_dev_ep_is_used(ept) ((ept)->ep != 0xFF)

/** Check if the endpoint is busy doing transactions. */
#define _usb_d_dev_ep_is_busy(ept) ((ept)->flags.bits.is_busy)

/** Check if the endpoint is control endpoint. */
#define _usb_d_dev_ep_is_ctrl(ept) ((ept)->flags.bits.eptype == USB_D_EPTYPE_CTRL)

/** Check if the endpoint use DMA. */
#define _usb_d_dev_ep_is_dma(epn) ((((epn) >= 1) && ((epn) < 7)) ? true : false)

/** Check if the endpoint transactions are IN. */
#define _usb_d_dev_ep_is_in(ept) ((ept)->flags.bits.dir)

/** Check if the endpoint transactions are OUT. */
#define _usb_d_dev_ep_is_out(ept) (!(ept)->flags.bits.dir)

/** Interrupt flags for WAKEUP event. */
#define USB_D_WAKEUP_INT_FLAGS (USBHS_DEVISR_UPRSM | USBHS_DEVISR_EORSM | USBHS_DEVISR_WAKEUP)

/** Interrupt flags for SUSPEND event. */
#define USB_D_SUSPEND_INT_FLAGS (USBHS_DEVISR_SUSP)

/** Max data bytes for a single DMA transfer. */
#define USB_D_DEV_TRANS_MAX 0x10000

/** Endpoint type setting to disable. */
#define USB_D_EPTYPE_DISABLE 0

/** Endpoint type setting to work as control endpoint. */
#define USB_D_EPTYPE_CTRL 1

/** Endpoint type setting to work as isochronous endpoint. */
#define USB_D_EPTYPE_ISOCH 2

/** Endpoint type setting to work as interrupt endpoint. */
#define USB_D_EPTYPE_INT 3

/** Endpoint type setting to work as bulk endpoint. */
#define USB_D_EPTYPE_BULK 4

/** Endpoint type setting for single bank endpoint. */
#define USB_D_EPTYPE_SINGLE 0

/** Endpoint type setting for dual bank endpoint. */
#define USB_D_EPTYPE_DUAL 1

/** Endpoint type setting for triple bank endpoint. */
#define USB_D_EPTYPE_TRIPLE 2

/** Endpoint bank type for control. */
#define USB_D_BANK_CTRL USB_D_EPTYPE_SINGLE

/** Endpoint bank type for isochronous. */
#define USB_D_BANK_ISOCH USB_D_EPTYPE_DUAL

/** Endpoint bank type for interrupt. */
#define USB_D_BANK_INT USB_D_EPTYPE_SINGLE

/** Endpoint bank type for bulk. */
#define USB_D_BANK_BULK USB_D_EPTYPE_DUAL

/** EPCFG register value for control endpoints. */
#define USB_D_EPCFG_CTRL 0

/** HPL USB device struct. */
struct _usb_d_dev {
	/** Callbacks of USB device. */
	struct _usb_d_dev_callbacks callbacks;
	/** Endpoint transaction callbacks. */
	struct _usb_d_dev_ep_callbacks ep_callbacks;
	/** Endpoints (ep0 + others). */
	struct _usb_d_dev_ep ep[USB_D_N_EP];
};
/*@}*/

/** Get 64-, 32-, 16- or 8-bit access to FIFO data register of selected endpoint.
 * \param[in] epn   Endpoint of which to access FIFO data register
 * \param[in] scale Data scale in bits: 64, 32, 16 or 8
 * \return Volatile 64-, 32-, 16- or 8-bit data pointer to FIFO data register
 * \note It is up to the user of this macro to make sure that all accesses
 *       are aligned with their natural boundaries except 64-bit accesses which
 *       require only 32-bit alignment.
 *       It is up to the user of this macro to make sure that used HSB
 *       addresses are identical to the DPRAM internal pointer modulo 32 bits.
 */
#define _usbd_ep_get_fifo_access(epn, scale)                                                                           \
	(((volatile uint##scale##_t(*)[USBHS_RAM_EP_SIZE / ((scale) / 8)]) USBHS_RAM_ADDR)[(epn)])

/** USB device driver instance. */
static struct _usb_d_dev dev_inst;

static void _usb_d_dev_reset_epts(void);

static void _usb_d_dev_trans_done(struct _usb_d_dev_ep *ept, const int32_t code);
static void _usb_d_dev_trans_stop(struct _usb_d_dev_ep *ept, bool dir, const int32_t code);

static void _usb_d_dev_in_next(struct _usb_d_dev_ep *ept);
static void _usb_d_dev_out_next(struct _usb_d_dev_ep *ept);
#if (CONF_USB_D_DMA_ENABLE == 1)
static void _usb_d_dev_dma_next(struct _usb_d_dev_ep *ept);
#endif

static inline void _usb_d_dev_trans_setup(struct _usb_d_dev_ep *ept);

/** \brief Enable/disable the endpoint
 * \param[in] epn Endpoint number.
 * \param[in] flags Interrupt flags.
 */
static inline void _usbd_ep_enable(uint8_t epn, uint32_t flags)
{
	uint32_t data;
	Usbhs *  hw = USBHS;

	data = hri_usbhs_read_DEVEPT_reg(hw);

	if (flags) {
		data |= 0x1 << epn;
	} else {
		data &= ~((uint32_t)0x1 << epn);
	}

	hri_usbhs_write_DEVEPT_reg(hw, data);
}

/** \brief ACK the endpoint interrupt
 * \param[in] epn Endpoint number.
 * \param[in] flags Interrupt flags.
 */
static inline void _usbd_ep_int_ack(uint8_t epn, uint32_t flags)
{
	hri_usbhs_write_DEVEPTICR_reg(USBHS, epn, flags);
}

/** \brief Enable the endpoint interrupt
 * \param[in] epn Endpoint number.
 * \param[in] flags Interrupt flags.
 */
static inline void _usbd_ep_int_en(uint8_t epn, uint32_t flags)
{
	hri_usbhs_write_DEVEPTIER_reg(USBHS, epn, flags);
	hri_usbhs_set_DEVIMR_reg(USBHS, USBHS_DEVIMR_PEP_0 << epn);
}

/** \brief Disable the endpoint interrupt
 * \param[in] epn Endpoint number.
 * \param[in] flags Interrupt flags.
 */
static inline void _usbd_ep_int_dis(uint8_t epn, uint32_t flags)
{
	hri_usbhs_write_DEVEPTIDR_reg(USBHS, epn, flags);
}

/** \brief Check if endpoint is control endpoint
 * \param[in] epn Endpoint number.
 */
static inline bool _usbd_ep_is_ctrl(uint8_t epn)
{
	return (hri_usbhs_read_DEVEPTCFG_EPTYPE_bf(USBHS, epn) == USB_D_EPCFG_CTRL);
}

/** \brief Check if endpoint is enabled
 * \param[in] epn Endpoint number.
 */
static inline bool _usbd_ep_is_enabled(uint8_t epn)
{
	uint32_t data;

	data = hri_usbhs_read_DEVEPT_reg(USBHS) & USBHS_DEVEPT_EPEN_Msk;
	return (data & (0x1 << epn));
}

/** \brief Check if endpoint is Configuration OK
 * \param[in] epn Endpoint number.
 */
static inline bool _usbd_ep_is_cfg(uint8_t epn)
{
	return hri_usbhs_get_DEVEPTISR_CFGOK_bit(USBHS, epn);
}

/** \brief Set endpoint stall
 * \param[in] epn Endpoint number.
 * \param[in] st Stall status.
 */
static inline void _usbd_ep_set_stall(uint8_t epn, bool st)
{
	Usbhs *hw = USBHS;

	if (st) {
		hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_STALLRQS);
	} else {
		hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_STALLRQC);
	}
}

/** \brief Check if the endpoint is stalled
 * \param[in] epn Endpoint number.
 * \return \c true if it's stalled.
 */
static inline bool _usbd_ep_is_stalled(uint8_t epn)
{
	Usbhs *hw = USBHS;

	return (hri_usbhs_get_DEVEPTISR_STALLEDI_bit(hw, epn) & hri_usbhs_get_DEVEPTIMR_STALLEDE_bit(hw, epn));
}

/** \brief Check if stall has been sent from the endpoint
 * \param[in] epn Endpoint number.
 * \return \c true if it's sent.
 */
static inline bool _usbd_ep_is_stall_sent(uint8_t epn)
{
	return (hri_usbhs_get_DEVEPTISR_STALLEDI_bit(USBHS, epn));
}

/** \brief ACK endpoint STALL interrupt
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_ack_stall(uint8_t epn)
{
	_usbd_ep_int_ack(epn, USBHS_DEVEPTISR_STALLEDI);
}

/** \brief Enable/disable endpoint STALL interrupt
 * \param[in] epn Endpoint number.
 * \param[in] en \c true to enable, \c false to disable.
 */
static inline void _usbd_ep_int_stall_en(uint8_t epn, const bool en)
{
	if (en) {
		_usbd_ep_int_en(epn, USBHS_DEVEPTIMR_STALLEDE);
	} else {
		_usbd_ep_int_dis(epn, USBHS_DEVEPTIMR_STALLEDE);
	}
}

/** \brief Stop SETUP transactions
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_stop_setup(uint8_t epn)
{
	hri_usbhs_write_DEVEPTIDR_reg(USBHS, epn, USBHS_DEVEPTIDR_RXSTPEC);
}

/** \brief Check if SETUP packet is ready
 * \param[in] epn Endpoint number.
 */
static inline bool _usbd_ep_is_setup(uint8_t epn)
{
	return hri_usbhs_get_DEVEPTISR_RXSTPI_bit(USBHS, epn);
}

/** \brief ACK endpoint SETUP interrupt
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_ack_setup(uint8_t epn)
{
	_usbd_ep_int_ack(epn, USBHS_DEVEPTISR_RXSTPI);
}

/** \brief Reset endpoint without configurations
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_reset(uint8_t epn)
{
	Usbhs *hw = USBHS;
	hri_usbhs_set_DEVEPT_reg(hw, USBHS_DEVEPT_EPRST0 << epn);
	hri_usbhs_clear_DEVEPT_reg(hw, USBHS_DEVEPT_EPRST0 << epn);
}

/** \brief Reset endpoint toggle value
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_reset_toggle(uint8_t epn)
{
	Usbhs *hw = USBHS;
	hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_RSTDTS);
}

/** \brief Set memory allocate
 * \param[in] epn Endpoint number.
 * \param[in] flags \c true to enable, \c false to disable.
 */
static inline void _usbd_ep_set_alloc(uint8_t epn, bool flags)
{
	Usbhs *hw = USBHS;

	if (flags) {
		hri_usbhs_set_DEVEPTCFG_ALLOC_bit(hw, epn);
	} else {
		hri_usbhs_clear_DEVEPTCFG_ALLOC_bit(hw, epn);
	}
}

/** \brief Get byte count for transactions
 * \param[in] epn Endpoint number.
 */
static inline uint16_t _usbd_ep_get_trans_count(uint8_t epn)
{
	return (hri_usbhs_get_DEVEPTISR_reg(USBHS, epn, USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos);
}

/** \brief Send a ZLP OUT on control endpoint
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_send_zlp_out(uint8_t epn)
{
	Usbhs *hw = USBHS;

	/* No action is necessary to accept OUT ZLP because the buffer of control
	   endpoint is already free. To detect a protocol error, enable nak
	   interrupt on data IN phase. */
	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_NAKINIC);
	hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_NAKINES);
}

/** \brief Send a ZLP IN on control endpoint
 * \param[in] epn Endpoint number.
 */
static inline void _usbd_ep_send_zlp_in(uint8_t epn)
{
	Usbhs *hw = USBHS;

	/* Send ZLP on IN endpoint. */
	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_TXINIC);

	/* To detect a protocol error, enable nak interrupt on data OUT phase. */
	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_NAKOUTIC);
	hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_NAKOUTES);
}

/** Set IN ready for IN transactions
 * \param[in] epn Endpoint number.
 * \param[in] rdy Set to \c true to indicate IN packet ready to TX.
 */
static inline void _usbd_ep_set_in_rdy(uint8_t epn, const bool rdy)
{
	Usbhs *hw = USBHS;

	if (rdy) {
		hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_TXINIC);
	} else {
		hri_usbhs_write_DEVEPTIFR_reg(hw, epn, USBHS_DEVEPTIFR_TXINIS);
	}
}

/** \brief Set ready for OUT transactions
 * \param[in] epn Endpoint number.
 * \param[in] rdy Set to \c true to indicate OUT bank ready to RX.
 */
static inline void _usbd_ep_set_out_rdy(uint8_t epn, const bool rdy)
{
	Usbhs *hw = USBHS;

	if (rdy) {
		hri_usbhs_write_DEVEPTIFR_reg(hw, epn, USBHS_DEVEPTIFR_RXOUTIS);
	} else {
		hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_RXOUTIC);
	}
}

/**
 *  \brief Convert USB endpoint size to HW PCKSIZE.SIZE
 * \param[in] n Number of bytes of endpoint size.
 */
static inline uint8_t _usbd_ep_pcksize_size(uint16_t n)
{
	return (
	    (n > 512)
	        ? 7
	        : ((n > 256) ? 6 : ((n > 128) ? 5 : ((n > 64) ? 4 : ((n > 32) ? 3 : ((n > 16) ? 2 : ((n > 8) ? 1 : 0)))))));
}

/**
 * \brief Handles USB SOF interrupt
 */
static inline void _usb_d_dev_sof(void)
{
	/* ACK SOF interrupt. */
	hri_usbhs_write_DEVICR_reg(USBHS, USBHS_DEVICR_SOFC);
	dev_inst.callbacks.sof();
}

/**
 * \brief Handles USB MSOF interrupt
 */
static inline void _usb_d_dev_msof(void)
{
	/* ACK SOF interrupt. */
	hri_usbhs_write_DEVICR_reg(USBHS, USBHS_DEVICR_MSOFC);
	dev_inst.callbacks.sof();
}

/**
 * \brief Handles USB resume/wakeup interrupts
 */
static inline void _usb_d_dev_wakeup(void)
{
	Usbhs *hw = USBHS;

	hri_usbhs_write_DEVICR_reg(hw, USB_D_WAKEUP_INT_FLAGS);
	hri_usbhs_clear_DEVIMR_reg(hw, USB_D_WAKEUP_INT_FLAGS);
	hri_usbhs_set_DEVIMR_SUSPE_bit(hw);

	dev_inst.callbacks.event(USB_EV_WAKEUP, 0);
}

/**
 * \brief Handles USB signal reset interrupt
 */
static inline void _usb_d_dev_reset(void)
{
	Usbhs *hw = USBHS;

	hri_usbhs_write_DEVICR_reg(hw, USBHS_DEVICR_EORSTC);
	hri_usbhs_clear_DEVIMR_reg(hw, USB_D_WAKEUP_INT_FLAGS);
	hri_usbhs_set_DEVIMR_reg(hw, USB_D_SUSPEND_INT_FLAGS);

	_usb_d_dev_reset_epts();
	dev_inst.callbacks.event(USB_EV_RESET, 0);
}

static inline void _usb_d_dev_suspend(void)
{
	Usbhs *hw = USBHS;

	hri_usbhs_write_DEVICR_reg(hw, USB_D_SUSPEND_INT_FLAGS);
	hri_usbhs_clear_DEVIMR_reg(USBHS, USB_D_SUSPEND_INT_FLAGS);
	hri_usbhs_set_DEVIMR_reg(USBHS, USB_D_WAKEUP_INT_FLAGS);

	dev_inst.callbacks.event(USB_EV_SUSPEND, 0);
}

/**
 * \brief Handles USB non-endpoint interrupt
 */
static inline bool _usb_d_dev_handle_nep(void)
{
	bool     rc    = true;
	uint16_t flags = hri_usbhs_read_DEVISR_reg(USBHS);
	flags &= hri_usbhs_read_DEVIMR_reg(USBHS);

	if (flags & USBHS_DEVISR_SOF) {
		_usb_d_dev_sof();
		return true;
	} else if (flags & USBHS_DEVISR_MSOF) {
		_usb_d_dev_msof();
		return true;
	}

	if (flags & USB_D_WAKEUP_INT_FLAGS) {
		_usb_d_dev_wakeup();
	} else if (flags & USBHS_DEVISR_EORST) {
		_usb_d_dev_reset();
	} else if (flags & USBHS_DEVISR_SUSP) {
		_usb_d_dev_suspend();
	} else {
		rc = false;
	}

	return rc;
}

/**
 * \brief Prepare next IN transactions
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_in_next(struct _usb_d_dev_ep *ept)
{
	Usbhs *  hw       = USBHS;
	uint8_t  epn      = USB_EP_GET_N(ept->ep);
	uint8_t *ptr_dest = (uint8_t *)&_usbd_ep_get_fifo_access(epn, 8);
	uint8_t *ptr_src;

	uint16_t trans_count = ept->trans_load;
	uint16_t trans_next;
	uint16_t last_pkt = trans_count & ((ept->size == 1023) ? ept->size : (ept->size - 1));
	bool     is_ctrl  = _usb_d_dev_ep_is_ctrl(ept);

	if (ept->trans_count >= ept->trans_size) {
		if (ept->flags.bits.need_zlp) {
			ept->trans_load          = 0;
			ept->flags.bits.need_zlp = 0;
			_usbd_ep_send_zlp_in(epn);
			if (!is_ctrl) {
				/** Switch to next bank. */
				hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_FIFOCONC);
			}
			return;
		}

		/* Complete. */
		hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_TXINEC);
		if (!is_ctrl) {
			hri_usbhs_clear_DEVIMR_reg(hw, (USBHS_DEVIER_PEP_0 << epn));
		}

		ept->trans_size = ept->trans_count;

		/* No ping-pong, so ask more data without background transfer. */
		if (last_pkt == ept->size) {
			ept->flags.bits.is_busy = 0;
			if (dev_inst.ep_callbacks.more(ept->ep, ept->trans_count)) {
				/* More data added. */
				return;
			}
			ept->flags.bits.is_busy = 1;
		}
		/* Finish normally. */
		_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
		return;
	} else {
		trans_next = ept->trans_size - ept->trans_count;
		if (trans_next > ept->size) {
			trans_next = ept->size;
		}
		ptr_src = &ept->trans_buf[ept->trans_count];
		memcpy(ptr_dest, ptr_src, trans_next);
		ept->trans_load = trans_next;
		ept->trans_count += trans_next;
		hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_TXINIC);
		if (!is_ctrl) {
			/** Switch to next bank. */
			hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_FIFOCONC);
		}
	}
}

/**
 * \brief Prepare next OUT transactions
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_out_next(struct _usb_d_dev_ep *ept)
{
	Usbhs *  hw          = USBHS;
	uint8_t  epn         = USB_EP_GET_N(ept->ep);
	uint16_t last_trans  = _usbd_ep_get_trans_count(epn);
	uint16_t last_remain = ept->trans_size - ept->trans_count;
	uint8_t *ptr         = (uint8_t *)&_usbd_ep_get_fifo_access(epn, 8);
	bool     is_full = false, is_short = false;
	bool     is_ctrl = _usb_d_dev_ep_is_ctrl(ept);

	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_RXOUTIC);

	if (last_trans > 0) {
		if (last_trans > last_remain) {
			last_trans = last_remain;
			is_full    = true;
		}
		memcpy(&ept->trans_buf[ept->trans_count], ptr, last_trans);
		ept->trans_count += last_trans;
		ept->trans_load = last_trans;
		hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_FIFOCONC);
	}

	if (last_trans < ept->size) {
		ept->flags.bits.need_zlp = 0;
		is_short                 = true;
	} else if (ept->trans_count >= ept->trans_size) {
		is_full = true;
	}

	/* Complete. */
	if (is_full || is_short) {
		hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_RXOUTEC);
		if (!is_ctrl) {
			hri_usbhs_clear_DEVIMR_reg(hw, (USBHS_DEVIER_PEP_0 << epn));
		}
		ept->trans_size = ept->trans_count;
		_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
	}
}

#if (CONF_USB_D_DMA_ENABLE == 1)
/**
 * \brief Prepare next OUT transactions
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_dma_next(struct _usb_d_dev_ep *ept)
{
	Usbhs *  hw  = USBHS;
	uint8_t  epn = USB_EP_GET_N(ept->ep);
	uint32_t trans_next;
	uint32_t dma_ctrl;

	if (!ept->flags.bits.is_busy) {
		/* No job is running, then ignore it (system error) */
		return;
	}

	if (ept->trans_count != ept->trans_size) {
		trans_next = ept->trans_size - ept->trans_count;
		if (trans_next > USB_D_DEV_TRANS_MAX) {
			trans_next = USB_D_DEV_TRANS_MAX;
			/* Set 0 to transfer the maximum */
			dma_ctrl = USBHS_DEVDMACONTROL_BUFF_LENGTH(0);
		} else {
			dma_ctrl = USBHS_DEVDMACONTROL_BUFF_LENGTH(trans_next);
		}
		if ((ept->flags.bits.dir) && (!ept->flags.bits.need_zlp)) {
			/* Enable short packet option, else the DMA transfer is accepted
			    and interrupt DMA valid but nothing is sent. */
			dma_ctrl |= USBHS_DEVDMACONTROL_END_B_EN;
		} else {
			if ((ept->flags.bits.eptype != USB_D_EPTYPE_ISOCH) || (trans_next <= ept->size)) {
				/* Enable short packet reception */
				dma_ctrl |= USBHS_DEVDMACONTROL_END_TR_IT | USBHS_DEVDMACONTROL_END_TR_EN;
			}
		}

		hri_usbhs_write_DEVDMAADDRESS_reg(hw, (epn - 1), (uint32_t)&ept->trans_buf[ept->trans_count]);
		dma_ctrl |= USBHS_DEVDMACONTROL_END_BUFFIT | USBHS_DEVDMACONTROL_CHANN_ENB;

		if (!hri_usbhs_get_DEVDMASTATUS_END_TR_ST_bit(hw, (epn - 1))) {
			hri_usbhs_write_DEVDMACONTROL_reg(hw, (epn - 1), dma_ctrl);
			ept->trans_count += trans_next;
			ept->trans_load = trans_next;
			hri_usbhs_set_DEVIMR_reg(hw, (USBHS_DEVIMR_DMA_1 << (epn - 1)));
			return;
		}

		/* Here a ZLP has been received and the DMA transfer must be not started.
		   It is the end of transfer. */
		ept->trans_size = ept->trans_count;
	}

	if ((ept->flags.bits.dir) && (ept->flags.bits.need_zlp)) {
		/* Need to send a ZLP (No possible with USB DMA) enable interrupt to
		   wait a free bank to sent ZLP. */
		_usbd_ep_int_ack(epn, USBHS_DEVEPTICR_TXINIC);
		if (hri_usbhs_get_DEVEPTISR_RWALL_bit(hw, epn)) {
			hri_usbhs_write_DEVEPTIFR_reg(hw, epn, USBHS_DEVEPTIFR_TXINIS);
		}
		hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_TXINES);
		hri_usbhs_set_DEVIMR_reg(hw, (USBHS_DEVIER_PEP_0 << epn));
		return;
	}

	/* Complete */
	_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
}
#endif

/**
 * \brief Handles setup received interrupt
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_handle_setup(struct _usb_d_dev_ep *ept)
{
	uint8_t epn     = USB_EP_GET_N(ept->ep);
	bool    is_ctrl = _usb_d_dev_ep_is_ctrl(ept);

	if (!is_ctrl) {
		/* Should never be here! */
		_usbd_ep_ack_setup(epn);
		_usbd_ep_stop_setup(epn);
		return;
	}

	/* Control transfer:
	 * SETUP transaction will terminate IN/OUT transaction,
	 * and start new transaction with received SETUP packet.
	 */
	if (_usb_d_dev_ep_is_busy(ept)) {
		ept->flags.bits.is_busy = 0;

		/* Stop transfer on either direction. */
		_usbd_ep_set_in_rdy(epn, false);
		_usbd_ep_set_out_rdy(epn, false);
	}

	ept->flags.bits.is_stalled = 0;

	/* Clear status and notify SETUP */
	_usbd_ep_int_ack(epn, USBHS_DEVEPTISR_NAKINI | USBHS_DEVEPTISR_NAKOUTI);
	_usbd_ep_int_dis(epn, USBHS_DEVEPTIMR_NAKINE | USBHS_DEVEPTIMR_NAKOUTE);
	/* Invoke callback. */
	dev_inst.ep_callbacks.setup(ept->ep);
}

/**
 * \brief Handles stall sent interrupt
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_handle_stall(struct _usb_d_dev_ep *ept)
{
	uint8_t epn = USB_EP_GET_N(ept->ep);
	/* Clear interrupt enable. Leave status there for status check. */
	_usbd_ep_int_stall_en(epn, false);
	_usb_d_dev_trans_done(ept, USB_TRANS_STALL);
}

/**
 * \brief Handles transaction overflow interrupt
 * \param[in] ept Pointer to endpoint information.
 */
static void _usb_d_dev_handle_overflow(struct _usb_d_dev_ep *ept)
{
	Usbhs * hw  = USBHS;
	uint8_t epn = USB_EP_GET_N(ept->ep);

	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_OVERFIC);
	_usb_d_dev_trans_stop(ept, _usb_d_dev_ep_is_in(ept), USB_TRANS_ERROR);
}

/**
 * \brief Analyze flags for setup transaction
 * \param[in] ept Pointer to endpoint information.
 * \param[in] flags Endpoint interrupt flags.
 */
static inline void _usb_d_dev_trans_setup_isr(struct _usb_d_dev_ep *ept, const uint8_t flags)
{
	/*
	 * SETPU is automatically ACKed by hardware
	 * OUT & IN should be set to NAK when checking SETUP
	 * No need to check OUT & IN status.
	 */
	if (flags & USBHS_DEVEPTISR_RXSTPI) {
		_usb_d_dev_handle_setup(ept);
	} else if (flags & USBHS_DEVEPTISR_STALLEDI) {
		_usb_d_dev_handle_stall(ept);
	}
}

/**
 * \brief Analyze flags for IN transactions
 * \param[in] ept Pointer to endpoint information.
 * \param[in] flags Endpoint interrupt flags.
 */
static inline void _usb_d_dev_trans_in_isr(struct _usb_d_dev_ep *ept, const uint8_t flags)
{
	/*
	 * Check IN flags
	 * If control endpoint, SETUP & OUT is checked to see if abort
	 */
	if (flags & USBHS_DEVEPTISR_TXINI) {
		_usb_d_dev_in_next(ept);
	} else if (_usb_d_dev_ep_is_ctrl(ept)) {
		/* Check OUT NAK
		 * Check SETUP
		 */
		if (flags & USBHS_DEVEPTISR_RXSTPI) {
			_usb_d_dev_handle_setup(ept);
		}
	}
}

/**
 * \brief Analyze flags for OUT transactions
 * \param[in] ept Pointer to endpoint information.
 * \param[in] flags Endpoint interrupt flags.
 */
static inline void _usb_d_dev_trans_out_isr(struct _usb_d_dev_ep *ept, const uint8_t flags)
{
	/*
	 * Check OUT flags.
	 * If control endpoint, SETUP & IN NAK is checked to see if abort
	 */
	if (flags & USBHS_DEVEPTISR_OVERFI) {
		_usb_d_dev_handle_overflow(ept);
	} else if (flags & USBHS_DEVEPTISR_RXOUTI) {
		_usb_d_dev_out_next(ept);
	} else if (_usb_d_dev_ep_is_ctrl(ept)) {
		/*
		 * Check SETUP
		 */
		if (flags & USBHS_DEVEPTISR_RXSTPI) {
			_usb_d_dev_handle_setup(ept);
		}
	}
}

/**
 * \brief Handles the endpoint interrupts.
 * \param[in] epint Endpoint interrupt summary (by bits).
 * \param[in] ept Pointer to endpoint information.
 */
static inline void _usb_d_dev_handle_eps(uint32_t epint, struct _usb_d_dev_ep *ept)
{
	Usbhs *hw = USBHS;

	uint8_t flags, mask;
	uint8_t epn = USB_EP_GET_N(ept->ep);

	if (!(epint & (1u << epn))) {
		return;
	}

	flags = hri_usbhs_read_DEVEPTISR_reg(hw, epn);
	mask  = hri_usbhs_read_DEVEPTIMR_reg(hw, epn);
	flags &= mask;

	if (flags) {
		if (flags & USBHS_DEVEPTISR_STALLEDI) {
			_usb_d_dev_handle_stall(ept);
		} else if (!_usb_d_dev_ep_is_busy(ept)) {
			_usb_d_dev_trans_setup_isr(ept, flags);
		} else if (_usb_d_dev_ep_is_in(ept)) {
			_usb_d_dev_trans_in_isr(ept, flags);
		} else {
			_usb_d_dev_trans_out_isr(ept, flags);
		}
	}
}

#if (CONF_USB_D_DMA_ENABLE == 1)
/**
 * \brief Handles the endpoint DMA interrupts.
 * \param[in] epint Endpoint DMA interrupt summary (by bits).
 * \param[in] ept Pointer to endpoint information.
 */
static inline void _usb_d_dev_handle_dma(uint32_t epint, struct _usb_d_dev_ep *ept)
{
	Usbhs *hw = USBHS;

	uint8_t  epn = USB_EP_GET_N(ept->ep);
	uint32_t trans_next;

	if (!(epint & (1u << (epn - 1)))) {
		return;
	}

	if (hri_usbhs_get_DEVDMASTATUS_CHANN_ENB_bit(hw, (epn - 1))) {
		/* Ignore EOT_STA interrupt */
		return;
	}
	hri_usbhs_clear_DEVIMR_reg(hw, (USBHS_DEVIMR_DMA_1 << (epn - 1)));
	trans_next = hri_usbhs_read_DEVDMASTATUS_BUFF_COUNT_bf(hw, (epn - 1));
	if (trans_next) {
		/* Transfer no complete (short packet or ZLP) then: Update number of
		   data transfered */
		ept->trans_count -= trans_next;
		/* Set transfer complete to stop the transfer */
		ept->trans_size = ept->trans_count;
	}
	_usb_d_dev_dma_next(ept);
}
#endif

/**
 * \brief USB device interrupt handler
 */
void USBHS_Handler(void)
{
	Usbhs *  hw = USBHS;
	uint8_t  i;
	uint32_t flags, ep_int, dma_int;

	flags = hri_usbhs_read_DEVISR_reg(hw) & hri_usbhs_read_DEVIMR_reg(hw);

	ep_int = (flags & USBHS_DEVISR_PEP__Msk) >> USBHS_DEVISR_PEP__Pos;
#if (CONF_USB_D_DMA_ENABLE == 1)
	dma_int = (flags & USBHS_DEVISR_DMA__Msk) >> USBHS_DEVISR_DMA__Pos;
#endif

#if (CONF_USB_D_DMA_ENABLE == 1)
	if ((ep_int == 0) && (dma_int == 0)) {
#else
	if (ep_int == 0) {
#endif
		if (_usb_d_dev_handle_nep()) {
			return;
		}
	}

	/* Handle endpoints */
	for (i = 0; i < USB_D_N_EP; i++) {
		struct _usb_d_dev_ep *ept = &dev_inst.ep[i];

		if (ept->ep == 0xFF) {
			continue;
		}
		if (ep_int) {
			_usb_d_dev_handle_eps(ep_int, ept);
		}
#if (CONF_USB_D_DMA_ENABLE == 1)
		else if ((dma_int) && (_usb_d_dev_ep_is_dma(i))) {
			_usb_d_dev_handle_dma(dma_int, ept);
		}
#endif
	}
}

/**
 * \brief Reset all endpoint software instances
 */
static void _usb_d_dev_reset_epts(void)
{
	uint8_t i;

	/* Reset USB address to 0 */
	hri_usbhs_clear_DEVCTRL_ADDEN_bit(USBHS);
	hri_usbhs_write_DEVCTRL_UADD_bf(USBHS, 0);
	hri_usbhs_set_DEVCTRL_ADDEN_bit(USBHS);

	for (i = 0; i < USB_D_N_EP; i++) {
		_usb_d_dev_trans_done(&dev_inst.ep[i], USB_TRANS_RESET);
		dev_inst.ep[i].ep       = 0xFF;
		dev_inst.ep[i].flags.u8 = 0;
	}
}

int32_t _usb_d_dev_init(void)
{
	Usbhs *        hw = USBHS;
	uint32_t       data;
	uint8_t        speed      = CONF_USB_D_SPEED;
	const uint32_t spdconf[4] = {
#if (CONF_USBHS_SRC == CONF_SRC_USB_480M)
		/* Normal mode */
		USBHS_DEVCTRL_LS,         /* LS */
		USBHS_DEVCTRL_SPDCONF(3), /* FS */
		USBHS_DEVCTRL_SPDCONF(0), /* HS */
		0                         /* Reserved */
	};
#elif (CONF_USBHS_SRC == CONF_SRC_USB_48M)
		/* Low power mode */
		(USBHS_DEVCTRL_SPDCONF(1) | USBHS_DEVCTRL_LS), /* LS */
		USBHS_DEVCTRL_SPDCONF(1),                      /* FS */
		0,                                             /* HS */
		0                                              /* Reserved */
	};
#endif

	if (hri_usbhs_get_CTRL_USBE_bit(hw)) {
		return ERR_DENIED;
	}

	dev_inst.callbacks.sof   = (_usb_d_dev_sof_cb_t)_dummy_func_no_return;
	dev_inst.callbacks.event = (_usb_d_dev_event_cb_t)_dummy_func_no_return;

	dev_inst.ep_callbacks.setup = (_usb_d_dev_ep_cb_setup_t)_dummy_func_no_return;
	dev_inst.ep_callbacks.more  = (_usb_d_dev_ep_cb_more_t)_dummy_func_return_false;
	dev_inst.ep_callbacks.done  = (_usb_d_dev_ep_cb_done_t)_dummy_func_no_return;

	_usb_d_dev_reset_epts();
	hri_usbhs_write_CTRL_reg(hw, USBHS_CTRL_UIMOD);

	data = spdconf[speed] | USBHS_DEVCTRL_DETACH;
	hri_usbhs_write_DEVCTRL_reg(hw, data);

	return ERR_NONE;
}

void _usb_d_dev_deinit(void)
{
	_usb_d_dev_disable();

	NVIC_DisableIRQ(USBHS_IRQn);
	NVIC_ClearPendingIRQ(USBHS_IRQn);
}

int32_t _usb_d_dev_enable(void)
{
	Usbhs *hw = USBHS;

	if (!(hri_usbhs_get_CTRL_USBE_bit(hw))) {
		hri_usbhs_clear_CTRL_FRZCLK_bit(hw);
		hri_usbhs_set_CTRL_USBE_bit(hw);
	}

	while (!hri_usbhs_get_SR_CLKUSABLE_bit(hw)) {
		/* Waiting for clock is stable */
	}

	hri_usbhs_set_DEVIMR_reg(hw,
	                         USBHS_DEVIER_SUSPES |
	                             /*USBHS_DEVIER_MSOFES |*/ USBHS_DEVIER_SOFES | USBHS_DEVIER_EORSTES
	                             | USBHS_DEVIER_WAKEUPES);
	NVIC_EnableIRQ(USBHS_IRQn);

	return ERR_NONE;
}

int32_t _usb_d_dev_disable(void)
{
	Usbhs *hw = USBHS;

	if (hri_usbhs_get_CTRL_USBE_bit(hw)) {
		hri_usbhs_clear_CTRL_USBE_bit(hw);
		hri_usbhs_set_CTRL_FRZCLK_bit(hw);
	}

	NVIC_DisableIRQ(USBHS_IRQn);
	hri_usbhs_clear_DEVIMR_reg(hw, USBHS_DEVIDR_MASK);

	return ERR_NONE;
}

void _usb_d_dev_attach(void)
{
	hri_usbhs_clear_DEVCTRL_DETACH_bit(USBHS);
}

void _usb_d_dev_detach(void)
{
	hri_usbhs_set_DEVCTRL_DETACH_bit(USBHS);
}

void _usb_d_dev_send_remotewakeup(void)
{
	uint32_t retry = CONF_USB_RMT_WKUP_RETRY;
	Usbhs *  hw    = USBHS;

	while (!(hri_usbhs_get_DEVISR_SUSP_bit(hw) && (retry--))) {
		hri_usbhs_set_DEVCTRL_RMWKUP_bit(hw);
	}
}

enum usb_speed _usb_d_dev_get_speed(void)
{
	uint32_t sp = hri_usbhs_read_SR_reg(USBHS);

	sp                            = (enum usb_speed)((sp & USBHS_SR_SPEED_Msk) >> USBHS_SR_SPEED_Pos);
	const enum usb_speed speed[3] = {USB_SPEED_FS, USB_SPEED_HS, USB_SPEED_LS};

	return speed[sp];
}

void _usb_d_dev_set_address(uint8_t addr)
{
	hri_usbhs_write_DEVCTRL_UADD_bf(USBHS, addr);
	hri_usbhs_set_DEVCTRL_ADDEN_bit(USBHS);
}

uint8_t _usb_d_dev_get_address(void)
{
	uint8_t addr = hri_usbhs_read_DEVCTRL_UADD_bf(USBHS);
	return addr;
}

uint16_t _usb_d_dev_get_frame_n(void)
{
	uint16_t fn = hri_usbhs_read_DEVFNUM_FNUM_bf(USBHS);
	return fn;
}

uint8_t _usb_d_dev_get_uframe_n(void)
{
	uint8_t ufn = hri_usbhs_read_DEVFNUM_MFNUM_bf(USBHS);
	return ufn;
}

/**
 *  \brief Start a setup transaction
 *  \param[in] ept Endpoint information.
 */
static inline void _usb_d_dev_trans_setup(struct _usb_d_dev_ep *ept)
{
	Usbhs * hw  = USBHS;
	uint8_t epn = USB_EP_GET_N(ept->ep);

	hri_usbhs_write_DEVEPTICR_reg(hw, epn, USBHS_DEVEPTICR_STALLEDIC | USBHS_DEVEPTICR_TXINIC);
	_usbd_ep_set_out_rdy(epn, false);

	hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_RXSTPES | USBHS_DEVEPTIER_RXOUTES);
	/* In case of abort of IN Data Phase:
	   No need to abort IN transfer (rise TXINI),
	   because it is automatically done by hardware when a Setup packet is received.
	   But the interrupt must be disabled to don't generate interrupt TXINI
	   after SETUP reception. */
	hri_usbhs_write_DEVEPTIDR_reg(hw, epn, USBHS_DEVEPTIDR_TXINEC);
	hri_usbhs_set_DEVIMR_reg(hw, (USBHS_DEVIER_PEP_0 << epn));
}

int32_t _usb_d_dev_ep0_init(const uint8_t max_pkt_siz)
{
	return _usb_d_dev_ep_init(0, USB_EP_XTYPE_CTRL, max_pkt_siz);
}

int32_t _usb_d_dev_ep_init(const uint8_t ep, const uint8_t attr, const uint16_t max_pkt_siz)
{
	uint8_t               bank = 0;
	uint32_t              data;
	uint8_t               epn = USB_EP_GET_N(ep);
	bool                  dir = USB_EP_GET_DIR(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	/* Check if endpoint size is 8,16,32,64,128,256,512 or 1023 */
	ASSERT(max_pkt_siz < 1024);
	ASSERT((max_pkt_siz == 1023) || !(max_pkt_siz & (max_pkt_siz - 1)));
	ASSERT(max_pkt_siz >= 8);

	uint8_t ep_type = attr & USB_EP_XTYPE_MASK;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}

	if (ept->ep != 0xFF) {
		return -USB_ERR_REDO;
	}

	switch (ep_type) {
	case USB_EP_XTYPE_CTRL:
		bank = USB_D_BANK_CTRL;
		break;

	case USB_EP_XTYPE_ISOCH:
		bank = USB_D_BANK_ISOCH;
		break;

	case USB_EP_XTYPE_BULK:
		bank = USB_D_BANK_BULK;
		break;

	case USB_EP_XTYPE_INTERRUPT:
		bank = USB_D_BANK_INT;
		break;
	default:
		ASSERT(false);
	}

	data = hri_usbhs_read_DEVEPTCFG_reg(USBHS, epn);
	data &= ~(USBHS_DEVEPTCFG_EPBK_Msk | USBHS_DEVEPTCFG_EPSIZE_Msk | USBHS_DEVEPTCFG_EPDIR
	          | USBHS_DEVEPTCFG_EPTYPE_Msk);
	data |= USBHS_DEVEPTCFG_EPBK(bank) | USBHS_DEVEPTCFG_EPSIZE(_usbd_ep_pcksize_size(max_pkt_siz))
	        | (((uint32_t)(dir) << USBHS_DEVEPTCFG_EPDIR_Pos) & USBHS_DEVEPTCFG_EPDIR)
	        | USBHS_DEVEPTCFG_EPTYPE(ep_type);
	hri_usbhs_write_DEVEPTCFG_reg(USBHS, epn, data);

	/* Initialize EP n settings */
	ept->size     = max_pkt_siz;
	ept->flags.u8 = (ep_type + 1);
	ept->ep       = ep;

	return USB_OK;
}

void _usb_d_dev_ep_deinit(uint8_t ep)
{
	Usbhs *               hw  = USBHS;
	uint8_t               epn = USB_EP_GET_N(ep);
	bool                  dir = USB_EP_GET_DIR(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	if (epn > CONF_USB_D_MAX_EP_N || !_usb_d_dev_ep_is_used(ept)) {
		return;
	}

	/* Finish pending transactions. */
	_usb_d_dev_trans_stop(ept, dir, USB_TRANS_RESET);

	/* Disable the endpoint. */
	hri_usbhs_clear_DEVEPT_reg(hw, USBHS_DEVEPT_EPEN0 << epn);
	ept->flags.u8 = 0;
	ept->ep       = 0xFF;
}

int32_t _usb_d_dev_ep_enable(const uint8_t ep)
{
	Usbhs *               hw           = USBHS;
	uint8_t               epn          = USB_EP_GET_N(ep);
	bool                  dir          = USB_EP_GET_DIR(ep);
	struct _usb_d_dev_ep *ept          = &dev_inst.ep[epn];
	uint16_t              ep_allocated = 1 << epn;
	uint8_t               i;

	if (epn > CONF_USB_D_MAX_EP_N || !_usb_d_dev_ep_is_used(ept)) {
		return -USB_ERR_PARAM;
	}

	if (epn == 0) {
		_usbd_ep_set_alloc(0, true);
		_usbd_ep_enable(0, true);

		if (!_usbd_ep_is_cfg(0)) {
			return -USB_ERR_ALLOC_FAIL;
		}
	} else {
		/* Un-alloc endpoints superior. */
		for (i = CONF_USB_D_MAX_EP_N; i > epn; i--) {
			if (_usbd_ep_is_enabled(i)) {
				ep_allocated |= 1 << i;
				_usbd_ep_enable(i, false);
				_usbd_ep_set_alloc(i, false);
			}
		}
		/* Re-alloc/Enable endpoints */
		for (i = epn; i <= CONF_USB_D_MAX_EP_N; i++) {
			if (ep_allocated & (1 << i)) {
				struct _usb_d_dev_ep *ptr_ep    = &dev_inst.ep[i];
				bool                  b_restart = ptr_ep->flags.bits.is_busy;
				ptr_ep->flags.bits.is_busy      = false;
				_usbd_ep_set_alloc(i, true);
				_usbd_ep_enable(i, true);

				if (!_usbd_ep_is_cfg(i)) {
					return -USB_ERR_ALLOC_FAIL;
				}

				hri_usbhs_clear_DEVEPTCFG_AUTOSW_bit(hw, i);

				if (b_restart && (!_usb_d_dev_ep_is_dma(i) && !dir)) {
					ptr_ep->trans_count -= ptr_ep->trans_load;
				}
			}
		}
	}

	if (ept->flags.bits.eptype == USB_D_EPTYPE_CTRL) {
		/* By default, control endpoint accept SETUP and NAK all other token. */
		_usbd_ep_set_out_rdy(epn, false);
		_usbd_ep_set_in_rdy(epn, false);

		/* Enable SETUP reception for control endpoint. */
		_usb_d_dev_trans_setup(ept);
	} else if (dir) {
		/* By default, IN endpoint will NAK all token. */
		_usbd_ep_set_in_rdy(epn, false);
	} else {
		/* By default, OUT endpoint will NAK all token. */
		_usbd_ep_set_out_rdy(epn, false);
	}

	return USB_OK;
}

void _usb_d_dev_ep_disable(const uint8_t ep)
{
	Usbhs *               hw  = USBHS;
	uint8_t               epn = USB_EP_GET_N(ep);
	bool                  dir = USB_EP_GET_DIR(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	_usb_d_dev_trans_stop(ept, dir, USB_TRANS_RESET);
	hri_usbhs_clear_DEVEPT_reg(hw, (0x1 << epn));
}

/**
 * \brief Get endpoint stall status
 * \param[in] ept Pointer to endpoint information.
 * \return Stall status.
 * \retval \c true Endpoint is stalled.
 * \retval \c false Endpoint is not stalled.
 */
static inline int32_t _usb_d_dev_ep_stall_get(struct _usb_d_dev_ep *ept)
{
	return ept->flags.bits.is_stalled;
}

/**
 * \brief Set endpoint stall
 * \param[in, out] ept Pointer to endpoint information.
 * \return Always 0, success.
 */
static inline int32_t _usb_d_dev_ep_stall_set(struct _usb_d_dev_ep *ept)
{
	uint8_t epn = USB_EP_GET_N(ept->ep);
	if (epn > CONF_USB_D_MAX_EP_N) {
		return ERR_NOT_FOUND;
	}
	if (!ept->flags.bits.is_stalled) {
		ept->flags.bits.is_stalled = 1;
		_usbd_ep_int_en(epn, USBHS_DEVEPTIMR_STALLEDE);
		_usbd_ep_set_stall(epn, true);
	}

	/* In stall interrupt abort the transfer. */
	return ERR_NONE;
}

/**
 * \brief Clear endpoint stall
 * \param[in, out] ept Pointer to endpoint information.
 * \return Always 0, success.
 */
static inline int32_t _usb_d_dev_ep_stall_clr(struct _usb_d_dev_ep *ept)
{
	uint8_t epn = USB_EP_GET_N(ept->ep);

	if (!ept->flags.bits.is_stalled) {
		return ERR_NONE;
	}
	_usbd_ep_set_stall(epn, false);
	_usbd_ep_int_dis(epn, USBHS_DEVEPTIMR_STALLEDE);

	if (_usbd_ep_is_stall_sent(epn)) {
		_usbd_ep_ack_stall(epn);
		_usbd_ep_reset(epn);
		_usbd_ep_reset_toggle(epn);
	}

	if (_usb_d_dev_ep_is_ctrl(ept)) {
		if ((hri_usbhs_read_DEVEPTISR_reg(USBHS, epn) & USBHS_DEVEPTISR_STALLEDI) == 0) {
			ept->flags.bits.is_stalled = 0;
		}
	} else {
		ept->flags.bits.is_stalled = 0;
	}

	return ERR_NONE;
}

int32_t _usb_d_dev_ep_stall(const uint8_t ep, const enum usb_ep_stall_ctrl ctrl)
{
	uint8_t               epn = USB_EP_GET_N(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];
	int32_t               rc;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}

	if (USB_EP_STALL_SET == ctrl) {
		rc = _usb_d_dev_ep_stall_set(ept);
	} else if (USB_EP_STALL_CLR == ctrl) {
		rc = _usb_d_dev_ep_stall_clr(ept);
	} else {
		rc = _usb_d_dev_ep_stall_get(ept);
	}

	return rc;
}

/**
 *  \brief Finish the transaction and invoke callback
 * \param[in, out] ept Pointer to endpoint information.
 * \param[in] code Information code passed.
 */
static void _usb_d_dev_trans_done(struct _usb_d_dev_ep *ept, const int32_t code)
{
	if (!(_usb_d_dev_ep_is_used(ept) && _usb_d_dev_ep_is_busy(ept))) {
		return;
	}

	ept->flags.bits.is_busy = 0;
	dev_inst.ep_callbacks.done(ept->ep, code, ept->trans_count);
}

/**
 *  \brief Terminate the transaction with specific status code
 * \param[in, out] ept Pointer to endpoint information.
 * \param[in] dir Endpoint direction.
 * \param[in] code Information code passed.
 */
static void _usb_d_dev_trans_stop(struct _usb_d_dev_ep *ept, bool dir, const int32_t code)
{
	uint8_t epn = USB_EP_GET_N(ept->ep);

	if (!(_usb_d_dev_ep_is_used(ept) && _usb_d_dev_ep_is_busy(ept))) {
		return;
	}

	/* Stop transfer */
	if (dir) {
		/* NAK IN */
		_usbd_ep_set_in_rdy(epn, false);
	} else {
		/* NAK OUT */
		_usbd_ep_set_out_rdy(epn, false);
	}

	_usb_d_dev_trans_done(ept, code);
}

int32_t _usb_d_dev_ep_read_req(const uint8_t ep, uint8_t *req_buf)
{
	uint8_t epn = USB_EP_GET_N(ep);

	uint16_t bytes = _usbd_ep_get_trans_count(epn);

	if (epn > CONF_USB_D_MAX_EP_N || !req_buf) {
		return -USB_ERR_PARAM;
	}

	if (!_usbd_ep_is_ctrl(epn)) {
		return -USB_ERR_FUNC;
	}

	if (!_usbd_ep_is_setup(epn)) {
		return ERR_NONE;
	}

	uint8_t *ptr = (uint8_t *)&_usbd_ep_get_fifo_access(epn, 8);
	memcpy(req_buf, (void *)ptr, 8);

	_usbd_ep_ack_setup(epn);

	return bytes;
}

int32_t _usb_d_dev_ep_trans(const struct usb_d_transfer *trans)
{
	Usbhs *               hw  = USBHS;
	uint8_t               epn = USB_EP_GET_N(trans->ep);
	bool                  dir = USB_EP_GET_DIR(trans->ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	uint16_t size_mask      = (ept->size == 1023) ? 1023 : (ept->size - 1);
	bool     size_n_aligned = (trans->size & size_mask);
	bool     is_ctrl        = _usb_d_dev_ep_is_ctrl(ept);

	volatile hal_atomic_t flags;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}

	/* Check halt */
	if (ept->flags.bits.is_stalled) {
		return USB_HALTED;
	}

	/* Try to start transactions. */
	atomic_enter_critical(&flags);

	if (_usb_d_dev_ep_is_busy(ept)) {
		atomic_leave_critical(&flags);
		return USB_BUSY;
	}

	ept->flags.bits.is_busy = 1;
	atomic_leave_critical(&flags);

	/* Copy transaction information. */
	ept->trans_buf   = trans->buf;
	ept->trans_size  = trans->size;
	ept->trans_count = 0;
	ept->trans_load  = 0;

	ept->flags.bits.dir      = dir;
	ept->flags.bits.need_zlp = (trans->zlp && (!size_n_aligned));

#if (CONF_USB_D_DMA_ENABLE == 1)
	if (_usb_d_dev_ep_is_dma(epn)) {
		hri_usbhs_set_DEVEPTCFG_AUTOSW_bit(hw, epn);
		_usb_d_dev_dma_next(ept);
	} else {
#endif
		if (!is_ctrl) {
			hri_usbhs_set_DEVIMR_reg(hw, (USBHS_DEVIER_PEP_0 << epn));
		}
		hri_usbhs_clear_DEVEPTCFG_AUTOSW_bit(hw, epn);
		if (dir) {
			hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_TXINES);
		} else {
			hri_usbhs_write_DEVEPTIER_reg(hw, epn, USBHS_DEVEPTIER_RXOUTES);
		}
#if (CONF_USB_D_DMA_ENABLE == 1)
	}
#endif

	return ERR_NONE;
}

void _usb_d_dev_ep_abort(const uint8_t ep)
{
	uint8_t               epn = USB_EP_GET_N(ep);
	bool                  dir = USB_EP_GET_DIR(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	if (epn > CONF_USB_D_MAX_EP_N) {
		return;
	}

	_usb_d_dev_trans_stop(ept, dir, USB_TRANS_ABORT);
}

int32_t _usb_d_dev_ep_get_status(const uint8_t ep, struct usb_d_trans_status *stat)
{
	uint8_t               epn = USB_EP_GET_N(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];
	bool                  busy, stall;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return USB_ERR_PARAM;
	}

	busy  = ept->flags.bits.is_busy;
	stall = ept->flags.bits.is_stalled;

	if (stat) {
		stat->stall = stall;
		stat->busy  = busy;
		stat->setup = hri_usbhs_get_DEVEPTISR_RXSTPI_bit(USBHS, epn);
		stat->dir   = ept->flags.bits.dir;
		stat->size  = ept->trans_size;
		stat->count = ept->trans_count;
		stat->ep    = ep;
		stat->xtype = ept->flags.bits.eptype - 1;
	}

	if (stall) {
		return USB_HALTED;
	}

	if (busy) {
		return USB_BUSY;
	}

	return USB_OK;
}

void _usb_d_dev_register_callback(const enum usb_d_cb_type type, const FUNC_PTR func)
{
	FUNC_PTR f = (func == NULL) ? (FUNC_PTR)_dummy_func_no_return : (FUNC_PTR)func;

	if (type == USB_D_CB_EVENT) {
		dev_inst.callbacks.event = (_usb_d_dev_event_cb_t)f;
	} else if (type == USB_D_CB_SOF) {
		dev_inst.callbacks.sof = (_usb_d_dev_sof_cb_t)f;
	}
}

void _usb_d_dev_register_ep_callback(const enum usb_d_dev_ep_cb_type type, const FUNC_PTR func)
{
	FUNC_PTR f = (func == NULL) ? (FUNC_PTR)_dummy_func_no_return : (FUNC_PTR)func;

	if (type == USB_D_DEV_EP_CB_SETUP) {
		dev_inst.ep_callbacks.setup = (_usb_d_dev_ep_cb_setup_t)f;
	} else if (type == USB_D_DEV_EP_CB_MORE) {
		dev_inst.ep_callbacks.more = (_usb_d_dev_ep_cb_more_t)f;
	} else if (type == USB_D_DEV_EP_CB_DONE) {
		dev_inst.ep_callbacks.done = (_usb_d_dev_ep_cb_done_t)f;
	}
}

#undef DEPRECATED
