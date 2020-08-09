/**
 * \file
 *
 * \brief Control Area Network(CAN) functionality implementation.
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


#include "CanDriver.h"

#if SUPPORT_CAN

#include <hpl_can_config.h>
#include <cstring>

/**
 * \brief Retrieve pointer to parent structure
 */
#define CONTAINER_OF(ptr, type, field_name) ((type *)(((uint8_t *)ptr) - offsetof(type, field_name)))

// DC: these buffers must be within the first 64kB of RAM. So we now declare them in section CanMessage.
#ifdef CONF_CAN0_ENABLED
alignas(4) static uint8_t can0_rx_fifo[CONF_CAN0_F0DS * CONF_CAN0_RXF0C_F0S] __attribute__ ((section (".CanMessage")));
alignas(4) static uint8_t can0_tx_fifo[CONF_CAN0_TBDS * CONF_CAN0_TXBC_TFQS] __attribute__ ((section (".CanMessage")));
alignas(4) static struct _can_tx_event_entry can0_tx_event_fifo[CONF_CAN0_TXEFC_EFS] __attribute__ ((section (".CanMessage")));
alignas(4) static struct _can_standard_message_filter_element can0_rx_std_filter[CONF_CAN0_SIDFC_LSS] __attribute__ ((section (".CanMessage")));
alignas(4) static struct _can_extended_message_filter_element can0_rx_ext_filter[CONF_CAN0_XIDFC_LSS] __attribute__ ((section (".CanMessage")));

struct _can_context _can0_context = {.rx_fifo       = can0_rx_fifo,
                                     .tx_fifo       = can0_tx_fifo,
                                     .tx_event      = can0_tx_event_fifo,
                                     .rx_std_filter = can0_rx_std_filter,
                                     .rx_ext_filter = can0_rx_ext_filter};
static _can_async_device *_can0_dev = nullptr; /*!< Pointer to hpl device */

#endif /* CONF_CAN0_ENABLED */

#ifdef CONF_CAN1_ENABLED
alignas(4) static volatile uint8_t can1_rx_fifo[CONF_CAN1_F0DS * CONF_CAN1_RXF0C_F0S] __attribute__ ((section (".CanMessage")));
alignas(4) static volatile uint8_t can1_tx_fifo[CONF_CAN1_TBDS * CONF_CAN1_TXBC_TFQS] __attribute__ ((section (".CanMessage")));
alignas(4) static volatile _can_tx_event_entry can1_tx_event_fifo[CONF_CAN1_TXEFC_EFS] __attribute__ ((section (".CanMessage")));
alignas(4) static _can_standard_message_filter_element can1_rx_std_filter[CONF_CAN1_SIDFC_LSS] __attribute__ ((section (".CanMessage")));
alignas(4) static _can_extended_message_filter_element can1_rx_ext_filter[CONF_CAN1_XIDFC_LSS] __attribute__ ((section (".CanMessage")));

struct _can_context _can1_context = {.rx_fifo       = can1_rx_fifo,
                                     .tx_fifo       = can1_tx_fifo,
                                     .tx_event      = can1_tx_event_fifo,
                                     .rx_std_filter = can1_rx_std_filter,
                                     .rx_ext_filter = can1_rx_ext_filter};
static _can_async_device *_can1_dev = nullptr; /*!< Pointer to hpl device */

#endif /* CONF_CAN1_ENABLED */

static uint32_t messagesLost = 0;		// number of messages lost due to overflow of FIFO 0

uint32_t GetAndClearCanMessagesLost() noexcept
{
	const uint32_t ret = messagesLost;
	messagesLost = 0;
	return ret;
}

/**
 * \brief Initialize CAN.
 */
static int32_t _can_async_init(_can_async_device *const dev, Can *const hw, const CanTiming& timing) noexcept
{
	dev->hw = hw;

	hw->CCCR.reg |= CAN_CCCR_INIT;
	while ((hw->CCCR.reg & CAN_CCCR_INIT) == 0) { }
	hw->CCCR.reg |= CAN_CCCR_CCE;

	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2;

	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 32 && tseg2 <= 16 && jumpWidth <= 16)
		{
			break;
		}
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

	const uint32_t nbtp = CAN_NBTP_NBRP(prescaler - 1) | CAN_NBTP_NTSEG1(tseg1 - 1) | CAN_NBTP_NTSEG2(tseg2 - 1) | CAN_NBTP_NSJW(jumpWidth - 1);

#ifdef CONF_CAN0_ENABLED
	if (hw == CAN0)
	{
		_can0_dev    = dev;
		dev->context = (void *)&_can0_context;
		dev->hw->CCCR.reg |= (1 << CAN_CCCR_FDOE_Pos) | (CONF_CAN0_CCCR_BRSE << CAN_CCCR_BRSE_Pos);
		hri_can_write_MRCFG_reg(dev->hw, CONF_CAN0_MRCFG_REG);
		hri_can_write_NBTP_reg(dev->hw, nbtp);
		hri_can_write_DBTP_reg(dev->hw, CONF_CAN0_DBTP_REG);
		hri_can_write_TDCR_reg(dev->hw, 0);								// use just the measured transceiver delay
		hri_can_write_RXF0C_reg(dev->hw,								// configure receive FIFO 0
			  (0 << CAN_RXF0C_F0OM_Pos)									// blocking mode
			| CAN_RXF0C_F0WM(0)											// no watermark set
			| CAN_RXF0C_F0S(CONF_CAN0_RXF0C_F0S)						// number of entries
			| CAN_RXF0C_F0SA((uint32_t)can0_rx_fifo));					// address
		hri_can_write_RXESC_reg(dev->hw, CAN_RXESC_F0DS_DATA64);		// receive fifo 0 element size
		hri_can_write_TXESC_reg(dev->hw, CAN_TXESC_TBDS_DATA64);		// transmit element size
		hri_can_write_TXBC_reg(dev->hw,									// configure transmit buffers
			  CAN_TXBC_TFQS(CONF_CAN0_TXBC_TFQS)						// number of Tx fifo entries
			| CAN_TXBC_TBSA((uint32_t)can0_tx_fifo));					// address
		hri_can_write_TXEFC_reg(dev->hw, 								// configure Tx event fifo
			  CAN_TXEFC_EFWM(CONF_CAN0_TXEFC_EFWM)
			| CAN_TXEFC_EFS(CONF_CAN0_TXEFC_EFS)
			| CAN_TXEFC_EFSA((uint32_t)can0_tx_event_fifo));			// address
		hw->GFC.reg =
			  CAN_GFC_ANFS(CONF_CAN0_GFC_ANFS)
			| CAN_GFC_ANFE(CONF_CAN0_GFC_ANFE)
			| (CONF_CAN0_GFC_RRFS << CAN_GFC_RRFS_Pos)
		    | (CONF_CAN0_GFC_RRFE << CAN_GFC_RRFE_Pos);
		hw->SIDFC.reg = CAN_SIDFC_LSS(CONF_CAN0_SIDFC_LSS) | CAN_SIDFC_FLSSA((uint32_t)can0_rx_std_filter);
		hw->XIDFC.reg = CAN_XIDFC_LSE(CONF_CAN0_XIDFC_LSS) | CAN_XIDFC_FLESA((uint32_t)can0_rx_ext_filter);
		hw->XIDAM.reg = CAN_XIDAM_EIDM(0x1FFFFFFF);

		hw->IR.reg = 0;													// disable all interrupt sources
		hw->IR.reg = 0xFFFFFFFF;										// clear all interrupt sources
		hw->ILS.reg = 0;												// all interrupt sources assigned to interrupt line 0 for now
		hw->ILE.reg = 0;

		NVIC_DisableIRQ(CAN0_IRQn);
		NVIC_ClearPendingIRQ(CAN0_IRQn);
		NVIC_EnableIRQ(CAN0_IRQn);
		hw->ILE.reg = CAN_ILE_EINT0;									// enable interrupt line 0
	}
#endif

#ifdef CONF_CAN1_ENABLED
	if (hw == CAN1)
	{
		_can1_dev    = dev;
		dev->context = (void *)&_can1_context;
		hri_can_set_CCCR_reg(dev->hw, CONF_CAN1_CCCR_REG);
		hri_can_write_MRCFG_reg(dev->hw, CONF_CAN1_MRCFG_REG);
		hri_can_write_NBTP_reg(dev->hw, nbtp);
		hri_can_write_DBTP_reg(dev->hw, CONF_CAN1_DBTP_REG);
		hri_can_write_TDCR_reg(dev->hw, 0);								// use just the measured transceiver delay
		hri_can_write_RXF0C_reg(dev->hw,
			  (CONF_CAN1_RXF0C_F0OM << CAN_RXF0C_F0OM_Pos)
			| CAN_RXF0C_F0WM(CONF_CAN1_RXF0C_F0WM)
		    | CAN_RXF0C_F0S(CONF_CAN1_RXF0C_F0S)
			| CAN_RXF0C_F0SA((uint32_t)can1_rx_fifo));
		hri_can_write_RXESC_reg(dev->hw, CONF_CAN1_RXESC_REG);
		hri_can_write_TXESC_reg(dev->hw, CONF_CAN1_TXESC_REG);
		hri_can_write_TXBC_reg(dev->hw, CAN_TXBC_TFQS(CONF_CAN1_TXBC_TFQS) | CAN_TXBC_TBSA((uint32_t)can1_tx_fifo));
		hri_can_write_TXEFC_reg(dev->hw, CONF_CAN1_TXEFC_REG | CAN_TXEFC_EFSA((uint32_t)can1_tx_event_fifo));
		hri_can_write_GFC_reg(dev->hw, CONF_CAN1_GFC_REG);
		hri_can_write_SIDFC_reg(dev->hw, CONF_CAN1_SIDFC_REG | CAN_SIDFC_FLSSA((uint32_t)can1_rx_std_filter));
		hri_can_write_XIDFC_reg(dev->hw, CONF_CAN1_XIDFC_REG | CAN_XIDFC_FLESA((uint32_t)can1_rx_ext_filter));
		hri_can_write_XIDAM_reg(dev->hw, CONF_CAN1_XIDAM_REG);

		hw->IR.reg = 0xFFFFFFFF;										// clear all interrupt sources
		hw->ILE.reg = 0;

		NVIC_DisableIRQ(CAN1_IRQn);
		NVIC_ClearPendingIRQ(CAN1_IRQn);
		NVIC_EnableIRQ(CAN1_IRQn);
		hri_can_write_ILE_reg(dev->hw, CAN_ILE_EINT0);
	}
#endif

	/* Disable CCE to prevent Configuration Change */
	hri_can_clear_CCCR_CCE_bit(dev->hw);
	hri_can_clear_CCCR_INIT_bit(dev->hw);
	while (hri_can_get_CCCR_INIT_bit(dev->hw)) { }

	return ERR_NONE;
}

/**
 * \brief De-initialize CAN.
 */
static void _can_async_deinit(_can_async_device *const dev) noexcept
{
	if (dev->hw != nullptr)
	{
		hri_can_set_CCCR_INIT_bit(dev->hw);
		dev->hw = nullptr;
	}
}

/**
 * \brief Enable CAN
 */
static int32_t _can_async_enable(_can_async_device *const dev) noexcept
{
	hri_can_clear_CCCR_INIT_bit(dev->hw);
	return ERR_NONE;
}

/**
 * \brief Disable CAN
 */
static int32_t _can_async_disable(_can_async_device *const dev) noexcept
{
	hri_can_set_CCCR_INIT_bit(dev->hw);
	return ERR_NONE;
}

/**
 * \brief Read a CAN message
 */
static int32_t _can_async_read(_can_async_device *const dev, can_message *msg) noexcept
{
	if (!hri_can_read_RXF0S_F0FL_bf(dev->hw))
	{
		return ERR_NOT_FOUND;
	}

	hri_can_rxf0s_reg_t get_index = hri_can_read_RXF0S_F0GI_bf(dev->hw);
	volatile _can_rx_fifo_entry *f = nullptr;

#ifdef CONF_CAN0_ENABLED
	if (dev->hw == CAN0)
	{
		f = (_can_rx_fifo_entry *)(can0_rx_fifo + get_index * CONF_CAN0_F0DS);
	}
#endif
#ifdef CONF_CAN1_ENABLED
	if (dev->hw == CAN1)
	{
		f = (_can_rx_fifo_entry *)(can1_rx_fifo + get_index * CONF_CAN1_F0DS);
	}
#endif

	if (f == nullptr)
	{
		return ERR_NO_RESOURCE;
	}

	if (f->R0.bit.XTD == 1)
	{
		msg->fmt = CAN_FMT_EXTID;
		msg->id  = f->R0.bit.ID;
	}
	else
	{
		msg->fmt = CAN_FMT_STDID;
		/* A standard identifier is stored into ID[28:18] */
		msg->id = f->R0.bit.ID >> 18;
	}

	if (f->R0.bit.RTR == 1) {
		msg->type = CAN_TYPE_REMOTE;
	}

	static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
	msg->len = dlc2len[f->R1.bit.DLC];

	memcpy(msg->data, const_cast<uint8_t*>(f->data), msg->len);

	hri_can_write_RXF0A_F0AI_bf(dev->hw, get_index);

	return ERR_NONE;
}

/**
 * \brief Write a CAN message
 */
static int32_t _can_async_write(_can_async_device *const dev, struct can_message *msg) noexcept
{
	if (hri_can_get_TXFQS_TFQF_bit(dev->hw))
	{
		return ERR_NO_RESOURCE;
	}

	volatile _can_tx_fifo_entry *f = nullptr;
	hri_can_txfqs_reg_t put_index = hri_can_read_TXFQS_TFQPI_bf(dev->hw);

#ifdef CONF_CAN0_ENABLED
	if (dev->hw == CAN0)
	{
		f = (struct _can_tx_fifo_entry *)(can0_tx_fifo + put_index * CONF_CAN0_TBDS);
	}
#endif
#ifdef CONF_CAN1_ENABLED
	if (dev->hw == CAN1)
	{
		f = (_can_tx_fifo_entry *)(can1_tx_fifo + put_index * CONF_CAN1_TBDS);
	}
#endif
	if (f == nullptr)
	{
		return ERR_NO_RESOURCE;
	}

	if (msg->fmt == CAN_FMT_EXTID)
	{
		f->T0.val     = msg->id;
		f->T0.bit.XTD = 1;
	}
	else
	{
		/* A standard identifier is stored into ID[28:18] */
		f->T0.val = msg->id << 18;
	}

	memcpy(const_cast<uint8_t*>(f->data), msg->data, msg->len);

	if (msg->len <= 8)
	{
		f->T1.bit.DLC = msg->len;
	}
	else
	{
		constexpr size_t MessageLengths[] = { 12, 16, 20, 24, 32, 48, 64 };
		size_t i;
		for (i = 0; i < ARRAY_SIZE(MessageLengths) && msg->len > MessageLengths[i]; ++i) { }
		f->T1.bit.DLC = 0x09 + i;

		// Set any additional bytes we will be sending to zero
		if (msg->len < MessageLengths[i])
		{
			memset(const_cast<uint8_t*>(f->data) + msg->len, 0, MessageLengths[i] - msg->len);
		}
	}

	f->T1.bit.FDF = hri_can_get_CCCR_FDOE_bit(dev->hw);
	f->T1.bit.BRS = hri_can_get_CCCR_BRSE_bit(dev->hw);

	hri_can_write_TXBAR_reg(dev->hw, 1 << put_index);
	return ERR_NONE;
}

/**
 * \brief Set CAN Interrupt State
 */
static void _can_async_set_irq_state(_can_async_device *const dev, enum can_async_callback_type type, bool state) noexcept
{
	if (type == CAN_ASYNC_RX_CB)
	{
		hri_can_write_IE_RF0NE_bit(dev->hw, state);
		hri_can_write_IE_RF0LE_bit(dev->hw, state);
	}
	else if (type == CAN_ASYNC_TX_CB)
	{
		hri_can_write_IE_TCE_bit(dev->hw, state);
		hri_can_write_TXBTIE_reg(dev->hw, CAN_TXBTIE_MASK);
	}
	else if (type == CAN_ASYNC_IRQ_CB)
	{
		const uint32_t ie = hri_can_get_IE_reg(dev->hw, CAN_IE_RF0NE | CAN_IE_TCE);
		hri_can_write_IE_reg(dev->hw, ie | CONF_CAN0_IE_REG);
	}
}

/**
 * \brief Return number of read errors
 */
static uint8_t _can_async_get_rxerr(_can_async_device *const dev) noexcept
{
	return hri_can_read_ECR_REC_bf(dev->hw);
}

/**
 * \brief Return number of write errors
 */
static uint8_t _can_async_get_txerr(_can_async_device *const dev) noexcept
{
	return hri_can_read_ECR_TEC_bf(dev->hw);
}

/**
 * \brief Set CAN to the specified mode
 */
static int32_t _can_async_set_mode(_can_async_device *const dev, enum can_mode mode) noexcept
{
	hri_can_set_CCCR_INIT_bit(dev->hw);
	while (hri_can_get_CCCR_INIT_bit(dev->hw) == 0) { }
	hri_can_set_CCCR_CCE_bit(dev->hw);

	if (mode == CAN_MODE_MONITORING)
	{
		hri_can_set_CCCR_MON_bit(dev->hw);
	}
	else
	{
		hri_can_clear_CCCR_MON_bit(dev->hw);
	}

	/* Disable CCE to prevent Configuration Change */
	hri_can_clear_CCCR_CCE_bit(dev->hw);
	hri_can_clear_CCCR_INIT_bit(dev->hw);
	while (hri_can_get_CCCR_INIT_bit(dev->hw)) { }

	return ERR_NONE;
}

/**
 * \brief Set CAN to the specified mode
 */
static int32_t _can_async_set_filter(_can_async_device *const dev, uint8_t index, can_format fmt, can_filter *filter) noexcept
{
	_can_standard_message_filter_element *sf = &((_can_context *)dev->context)->rx_std_filter[index];
	_can_extended_message_filter_element *ef = &((_can_context *)dev->context)->rx_ext_filter[index];

	if (fmt == CAN_FMT_STDID)
	{
		if (filter == nullptr)
		{
			sf->S0.val = 0;
			return ERR_NONE;
		}
		sf->S0.val       = filter->mask;
		sf->S0.bit.SFID1 = filter->id;
		sf->S0.bit.SFT   = _CAN_SFT_CLASSIC;
		sf->S0.bit.SFEC  = _CAN_SFEC_STF0M;
	}
	else if (fmt == CAN_FMT_EXTID)
	{
		if (filter == nullptr)
		{
			ef->F0.val = 0;
			return ERR_NONE;
		}
		ef->F0.val      = filter->id;
		ef->F0.bit.EFEC = _CAN_EFEC_STF0M;
		ef->F1.val      = filter->mask;
		ef->F1.bit.EFT  = _CAN_EFT_CLASSIC;
	}

	return ERR_NONE;
}

/*
 * \brief CAN interrupt handler
 */

#ifdef CONF_CAN0_ENABLED

void CAN0_Handler() noexcept
{
	_can_async_device *dev = _can0_dev;
	uint32_t                  ir;
#if 1	//dc42
	while (((ir = hri_can_read_IR_reg(dev->hw)) & (CAN_IR_RF0N | CAN_IR_TC | CAN_IR_BO | CAN_IR_EW | CAN_IR_EP | CAN_IR_RF0L)) != 0)
	{
		hri_can_write_IR_reg(dev->hw, ir);
#else
	ir = hri_can_read_IR_reg(dev->hw);
#endif

		if (ir & CAN_IR_RF0N)
		{
			dev->cb.rx_done(dev);
		}

		if (ir & CAN_IR_TC)
		{
			dev->cb.tx_done(dev);
		}

		if (ir & CAN_IR_BO)
		{
			dev->cb.irq_handler(dev, CAN_IRQ_BO);
		}

		if (ir & CAN_IR_EW)
		{
			dev->cb.irq_handler(dev, CAN_IRQ_EW);
		}

		if (ir & CAN_IR_EP)
		{
			dev->cb.irq_handler(dev, hri_can_get_PSR_EP_bit(dev->hw) ? CAN_IRQ_EP : CAN_IRQ_EA);
		}

		if (ir & CAN_IR_RF0L)
		{
			++messagesLost;
			dev->cb.irq_handler(dev, CAN_IRQ_DO);
		}

#if 1	//dc42
	}
#else
	hri_can_write_IR_reg(dev->hw, ir);
#endif
}

#endif

#ifdef CONF_CAN1_ENABLED

void CAN1_Handler() noexcept
{
	_can_async_device *dev = _can1_dev;
	uint32_t                  ir;
#if 1	//dc42
	while (((ir = hri_can_read_IR_reg(dev->hw)) & (CAN_IR_RF0N | CAN_IR_TC | CAN_IR_BO | CAN_IR_EW | CAN_IR_EP | CAN_IR_RF0L)) != 0)
	{
		hri_can_write_IR_reg(dev->hw, ir);
#else
	ir = hri_can_read_IR_reg(dev->hw);
#endif

	if (ir & CAN_IR_RF0N)
	{
		dev->cb.rx_done(dev);
	}

	if (ir & CAN_IR_TC)
	{
		dev->cb.tx_done(dev);
	}

	if (ir & CAN_IR_BO)
	{
		dev->cb.irq_handler(dev, CAN_IRQ_BO);
	}

	if (ir & CAN_IR_EW)
	{
		dev->cb.irq_handler(dev, CAN_IRQ_EW);
	}

	if (ir & CAN_IR_EP)
	{
		dev->cb.irq_handler(dev, hri_can_get_PSR_EP_bit(dev->hw) ? CAN_IRQ_EP : CAN_IRQ_EA);
	}

	if (ir & CAN_IR_RF0L)
	{
		dev->cb.irq_handler(dev, CAN_IRQ_DO);
	}

#if 1	//dc42
	}
#else
	hri_can_write_IR_reg(dev->hw, ir);
#endif
}

#endif

/**
 * \internal Callback of CAN Message Write finished
 *
 * \param[in] dev The pointer to CAN device structure
 */
static void can_tx_done(_can_async_device *dev) noexcept;
/**
 * \internal Callback of CAN Message Read finished
 *
 * \param[in] dev The pointer to CAN device structure
 */
static void can_rx_done(_can_async_device *dev) noexcept;
/**
 * \internal Callback of CAN Interrupt
 *
 * \param[in] dev  The pointer to CAN device structure
 * \param[in] type Interrupt source type
 */
static void can_irq_handler(_can_async_device *dev, enum can_async_interrupt_type type) noexcept;

/**
 * \brief Initialize CAN.
 */
int32_t can_async_init(can_async_descriptor *descr, Can *hw, const CanTiming& timing) noexcept
{
	const int32_t rc = _can_async_init(&descr->dev, hw, timing);
	if (rc)
	{
		return rc;
	}
	descr->dev.cb.tx_done     = can_tx_done;
	descr->dev.cb.rx_done     = can_rx_done;
	descr->dev.cb.irq_handler = can_irq_handler;

	return ERR_NONE;
}

/**
 * \brief Deinitialize CAN.
 */
void can_async_deinit(can_async_descriptor *const descr) noexcept
{
	_can_async_deinit(&descr->dev);
}

/**
 * \brief Enable CAN
 */
int32_t can_async_enable(can_async_descriptor *const descr) noexcept
{
	return _can_async_enable(&descr->dev);
}

/**
 * \brief Disable CAN
 */
int32_t can_async_disable(can_async_descriptor *const descr) noexcept
{
	return _can_async_disable(&descr->dev);
}

/**
 * \brief Read a CAN message
 */
int32_t can_async_read(can_async_descriptor *const descr, struct can_message *msg) noexcept
{
	return _can_async_read(&descr->dev, msg);
}

/**
 * \brief Write a CAN message
 */
int32_t can_async_write(can_async_descriptor *const descr, struct can_message *msg) noexcept
{
	return _can_async_write(&descr->dev, msg);
}

/**
 * \brief Register CAN callback function to interrupt
 */
int32_t can_async_register_callback(can_async_descriptor *const descr, enum can_async_callback_type type, FUNC_PTR cb) noexcept
{
	switch (type) {
	case CAN_ASYNC_RX_CB:
		descr->cb.rx_done = (cb != nullptr) ? (can_cb_t)cb : nullptr;
		break;
	case CAN_ASYNC_TX_CB:
		descr->cb.tx_done = (cb != nullptr) ? (can_cb_t)cb : nullptr;
		break;
	case CAN_ASYNC_IRQ_CB:
		descr->cb.irq_handler
		    = (cb != nullptr) ? (void (*)(can_async_descriptor *const, enum can_async_interrupt_type) noexcept)cb : nullptr;
		break;
	default:
		return ERR_INVALID_ARG;
	}

	_can_async_set_irq_state(&descr->dev, type, nullptr != cb);

	return ERR_NONE;
}

/**
 * \brief Return number of read errors
 */
uint8_t can_async_get_rxerr(can_async_descriptor *const descr) noexcept
{
	return _can_async_get_rxerr(&descr->dev);
}

/**
 * \brief Return number of write errors
 */
uint8_t can_async_get_txerr(can_async_descriptor *const descr) noexcept
{
	return _can_async_get_txerr(&descr->dev);
}

/**
 * \brief Set CAN to the specified mode
 */
int32_t can_async_set_mode(can_async_descriptor *const descr, enum can_mode mode) noexcept
{
	return _can_async_set_mode(&descr->dev, mode);
}

/**
 * \brief Set CAN filter
 */
int32_t can_async_set_filter(can_async_descriptor *const descr, uint8_t index, enum can_format fmt, struct can_filter *filter) noexcept
{
	return _can_async_set_filter(&descr->dev, index, fmt, filter);
}

void GetLocalCanTiming(const can_async_descriptor *descr, CanTiming& timing) noexcept
{
	const uint32_t nbtp = descr->dev.hw->NBTP.reg;
	const uint32_t tseg1 = (nbtp & CAN_NBTP_NTSEG1_Msk) >> CAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (nbtp & CAN_NBTP_NTSEG2_Msk) >> CAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (nbtp & CAN_NBTP_NSJW_Msk) >> CAN_NBTP_NSJW_Pos;
	const uint32_t brp = (nbtp & CAN_NBTP_NBRP_Msk) >> CAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

/**
 * \internal Callback of CAN Message Write finished
 */
static void can_tx_done(_can_async_device *dev) noexcept
{
	can_async_descriptor *const descr = CONTAINER_OF(dev, can_async_descriptor, dev);

	if (descr->cb.tx_done)
	{
		descr->cb.tx_done(descr);
	}
}

/**
 * \internal Callback of CAN Message Read finished
 */
static void can_rx_done(_can_async_device *dev) noexcept
{
	can_async_descriptor *const descr = CONTAINER_OF(dev, can_async_descriptor, dev);

	if (descr->cb.rx_done)
	{
		descr->cb.rx_done(descr);
	}
}

/**
 * \internal Callback of CAN Interrupt
 */
static void can_irq_handler(_can_async_device *dev, can_async_interrupt_type type) noexcept
{
	can_async_descriptor *const descr = CONTAINER_OF(dev, can_async_descriptor, dev);

	if (descr->cb.irq_handler)
	{
		descr->cb.irq_handler(descr, type);
	}
}

#endif

// End
