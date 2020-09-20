/*
 * CanDevice.cpp
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#include "CanDevice.h"

#if 0	//SUPPORT_CAN

#include <CanSettings.h>
#include <General/Bitmap.h>

/**@}*/
/**
 * \brief CAN receive FIFO element.
 */
struct CanRxBufferHeader
{
	__IO union
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
	__IO union
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
};

template<size_t DataLength> struct CanRxBufferEntry : public CanRxBufferHeader
{
	uint8_t data[DataLength];
};

/**
 * \brief CAN transmit FIFO element.
 */
struct CanTxBufferHeader
{
	__IO union
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
	__IO union
	{
		struct
		{
			uint32_t : 16;    /*!< Reserved */
			uint32_t DLC : 4; /*!< Data Length Code */
			uint32_t BRS : 1; /*!< Bit Rate Switch */
			uint32_t FDF : 1; /*!< FD Format */
			uint32_t : 1;     /*!< Reserved */
			uint32_t EFC : 1; /*!< Event FIFO Control */
			uint32_t MM : 8;  /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T1;
};

template<size_t DataLength> struct CanTxBufferEntry : public CanTxBufferHeader
{
	uint8_t data[DataLength];
};

/**
 * \brief CAN transmit Event element.
 */
struct CanDevice::CanTxEventEntry
{
	__IO union
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
	__IO union
	{
		struct
		{
			uint32_t TXTS : 16; /*!< Tx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t ET : 2;    /*!< Event Type */
			uint32_t MM : 8;    /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;
};

/**
 * \brief CAN standard message ID filter element structure.
 *
 *  Common element structure for standard message ID filter element.
 */
struct CanDevice::CanStandardMessageFilterElement
{
	union S0Type
	{
		struct
		{
			uint32_t SFID2 : 11; /*!< Standard Filter ID 2 */
			uint32_t : 5;        /*!< Reserved */
			uint32_t SFID1 : 11; /*!< Standard Filter ID 1 */
			uint32_t SFEC : 3;   /*!< Standard Filter Configuration */
			uint32_t SFT : 2;    /*!< Standard Filter Type */
		} bit;
		uint32_t val; /*!< Type used for register access */
	};

	__IO S0Type S0;
};

/**
 * \brief CAN extended message ID filter element structure.
 *
 *  Common element structure for extended message ID filter element.
 */
struct CanDevice::CanExtendedMessageFilterElement
{
	union F0Type
	{
		struct
		{
			uint32_t EFID1 : 29;	//!< bit: Extended Filter ID 1
			uint32_t EFEC : 3;		//!< bit: Extended Filter Configuration
		} bit;
		uint32_t val;				//!< Type used for register access
	};

	union F1Type
	{
		struct
		{
			uint32_t EFID2 : 29;	//!< bit: Extended Filter ID 2
			uint32_t : 1;			//!< bit: Reserved
			uint32_t EFT : 2;		//!< bit: Extended Filter Type
		} bit;
		uint32_t val;				//!< Type used for register access
	};

	__IO union F0Type F0;
	__IO union F1Type F1;
};

struct CanDevice::CanContext
{
	CanRxBufferHeader *rx0Fifo;									//!< Receive message fifo
	CanRxBufferHeader *rx1Fifo;									//!< Receive message fifo
	CanRxBufferHeader *rxBuffers;								//!< Receive direct buffers
	CanTxBufferHeader *txBuffers;								//!< Transmit direct buffers
	CanTxBufferHeader *txFifo;									//!< Transfer message fifo
	CanTxEventEntry *txEventFifo;								//!< Transfer event fifo
	CanStandardMessageFilterElement *rxStdFilter;				//!< Standard filter List
	CanExtendedMessageFilterElement *rxExtFilter;				//!< Extended filter List
};

// DC: these buffers must be within the first 64kB of RAM. So we now declare them in section CanMessage.
constexpr size_t Can0DataSize = 64;
alignas(4) static CanRxBufferEntry<Can0DataSize> can0_rx_fifo0[CanDevice::RxFifo0Size[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanRxBufferEntry<Can0DataSize> can0_rx_fifo1[CanDevice::RxFifo1Size[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanRxBufferEntry<Can0DataSize> can0_rx_buffers[CanDevice::NumRxBuffers[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanTxBufferEntry<Can0DataSize> can0_tx_buffers[CanDevice::NumTxBuffers[0] + CanDevice::TxFifoSize[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanDevice::CanTxEventEntry can0_tx_event_fifo[CanDevice::TxEventFifoSize[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanDevice::CanStandardMessageFilterElement can0_rx_std_filter[CanDevice::NumShortFilterElements[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanDevice::CanExtendedMessageFilterElement can0_rx_ext_filter[CanDevice::NumExtendedFilterElements[0]] __attribute__ ((section (".CanMessage")));

#if SAME70
constexpr size_t Can1DataSize = 64;
alignas(4) static CanRxBufferEntry<Can1DataSize> can1_rx_fifo0[CanDevice::RxFifo0Size[1]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanRxBufferEntry<Can1DataSize> can1_rx_fifo1[CanDevice::RxFifo1Size[1]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanRxBufferEntry<Can1DataSize> can1_rx_buffers[CanDevice::NumRxBuffers[1]] __attribute__ ((section (".CanMessage")));
alignas(4) static CanTxBufferEntry<Can1DataSize> can1_tx_buffers[CanDevice::NumTxBuffers[1] + CanDevice::TxFifoSize[0]] __attribute__ ((section (".CanMessage")));
alignas(4) static _can_tx_event_entry can1_tx_event_fifo[CanDevice::TxEventFifoSize[1]] __attribute__ ((section (".CanMessage")));
alignas(4) static _can_standard_message_filter_element can1_rx_std_filter[CanDevice::NumShortFilterElements[1]] __attribute__ ((section (".CanMessage")));
alignas(4) static _can_extended_message_filter_element can1_rx_ext_filter[CanDevice::NumExtendedFilterElements[1]] __attribute__ ((section (".CanMessage")));
#endif

CanDevice::CanContext const CanDevice::CanContexts[CanDevice::NumCanDevices] =
{
	{
		.rx0Fifo = can0_rx_fifo0,
		.rx1Fifo = can0_rx_fifo1,
		.rxBuffers = can0_rx_buffers,
		.txBuffers = can0_tx_buffers,
		.txFifo = can0_tx_buffers + CanDevice::NumTxBuffers[0],
		.txEventFifo = can0_tx_event_fifo,
		.rxStdFilter = can0_rx_std_filter,
		.rxExtFilter = can0_rx_ext_filter
	},
#if SAME70
	{
		.rx0Fifo = can1_rx_fifo0,
		.rx1Fifo = can1_rx_fifo1,
		.rxBuffers = can1_rx_buffers,
		.txBuffers = can1_tx_buffers,
		.txFifo = can1_tx_buffers + CanDevice::NumTxBuffers[1],
		.txEventFifo = can1_tx_event_fifo,
		.rxStdFilter = can1_rx_std_filter,
		.rxExtFilter = can1_rx_ext_filter
	},
#endif
};

static Can * const CanPorts[2] = { CAN0, CAN1 };
static const IRQn IRQnsByPort[2] = { CAN0_IRQn, CAN1_IRQn };
static CanDevice *devicesByPort[2] = { nullptr, nullptr };

CanDevice CanDevice::devices[NumCanDevices];

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int whichCan, unsigned int whichPort, bool useFDMode, const CanTiming &timing) noexcept
{
	if (   whichCan >= NumCanDevices								// device number out of range
		|| whichPort >= 2											// CAN instance number out of range
		|| devicesByPort[whichPort] != nullptr						// CAN instance number already in use
	   )
	{
		return nullptr;
	}

	CanDevice& dev = devices[whichCan];
	if (dev.hw != nullptr)											// device instance already in use
	{
		return nullptr;
	}

	dev.whichCan = whichCan;
	dev.whichPort = whichPort;
	dev.hw = CanPorts[whichPort];
	devicesByPort[whichPort] = &dev;
	dev.context = &CanContexts[whichCan];
	dev.useFDMode = useFDMode;
	dev.messagesLost = dev.busOffCount = 0;
	dev.rxBuffersWaiting = dev.txBuffersWaiting = 0;
	dev.UpdateLocalCanTiming(timing);								// sets NBTP and DBTP
	dev.DoHardwareInit();
	return &dev;
}

// Do the low level hardware initialisation, excluding the bits relating to timing
void CanDevice::DoHardwareInit() noexcept
{
	hw->CCCR.reg |= CAN_CCCR_INIT;
	while ((hw->CCCR.reg & CAN_CCCR_INIT) == 0) { }

	hw->CCCR.reg |= CAN_CCCR_CCE;


	if (useFDMode)
	{
		hw->CCCR.reg |= CAN_CCCR_FDOE | CAN_CCCR_BRSE;
	}
	hw->MRCFG.reg = CAN_MRCFG_QOS_MEDIUM;
	hw->TDCR.reg = 0;											// use just the measured transceiver delay
	hw->RXF0C.reg = 											// configure receive FIFO 0
		  (0 << CAN_RXF0C_F0OM_Pos)									// blocking mode not overwrite mode
		| CAN_RXF0C_F0WM(0)											// no watermark interrupt
		| CAN_RXF0C_F0S(RxFifo0Size[whichCan])						// number of entries
		| CAN_RXF0C_F0SA((uint32_t)context->rx0Fifo);			// address
	hw->RXESC.reg =  CAN_RXESC_F0DS_DATA64;						// receive fifo 0 element size
	hw->TXESC.reg = CAN_TXESC_TBDS_DATA64;						// transmit element size
	hw->TXESC.reg = 											// configure transmit buffers
		  CAN_TXBC_TFQS(TxFifoSize[whichCan])						// number of Tx fifo entries
		| CAN_TXBC_TBSA((uint32_t)context->txFifo);				// address
	hw->TXESC.reg =  												// configure Tx event fifo
		  CAN_TXEFC_EFWM(0)
		| CAN_TXEFC_EFS(TxEventFifoSize[whichCan])
		| CAN_TXEFC_EFSA((uint32_t)context->txEventFifo);		// address
	hw->GFC.reg =
		  CAN_GFC_ANFS_REJECT
		| CAN_GFC_ANFE_REJECT
		| CAN_GFC_RRFS
		| CAN_GFC_RRFE;
	hw->SIDFC.reg = CAN_SIDFC_LSS(NumShortFilterElements[whichCan]) | CAN_SIDFC_FLSSA((uint32_t)context->rxStdFilter);
	hw->XIDAM.reg = CAN_XIDAM_EIDM(0x1FFFFFFF);

	hw->IR.reg = 0;												// disable all interrupt sources
	hw->IR.reg = 0xFFFFFFFF;									// clear all interrupt sources
	hw->ILS.reg = 0;											// all interrupt sources assigned to interrupt line 0 for now
	hw->ILE.reg = 0;

	const IRQn irqn = IRQnsByPort[whichPort];
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
	hw->ILE.reg = CAN_ILE_EINT0;								// enable interrupt line 0

	// Disable CCE to prevent Configuration Change
	hw->CCCR.reg &= ~CAN_CCCR_CCE;
	hw->CCCR.reg &= ~CAN_CCCR_INIT;
	while ((hw->CCCR.reg & CAN_CCCR_INIT) != 0) { }
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (hw != nullptr)
	{
		Disable();
		NVIC_DisableIRQ(IRQnsByPort[whichPort]);
		devicesByPort[whichPort] = nullptr;							// free the port
		hw = nullptr;												// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	hw->CCCR.reg &= ~CAN_CCCR_INIT;
}

// Disable this device
void CanDevice::Disable() noexcept
{
	hw->CCCR.reg |= CAN_CCCR_INIT;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
	TaskBase::ClearNotifyCount();
	{
		AtomicCriticalSectionLocker lock;

		if (whichBuffer == TxBufferNumber::fifo)
		{
			if (hw->TXFQS.bit.TFFL != 0)
			{
				return true;
			}
		}
		else if ((hw->TXBRP.reg & ((uint32_t)1 << (unsigned int)whichBuffer)) == 0)
		{
			return true;
		}

		if (timeout == 0)
		{
			return false;
		}

		txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
		txBuffersWaiting |= (uint32_t)1 << (unsigned int)whichBuffer;
	}

	TaskBase::Take(timeout);
	return (whichBuffer == TxBufferNumber::fifo)
				? (hw->TXFQS.bit.TFFL != 0)
					: (hw->TXBRP.reg & ((uint32_t)1 << ((unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0))) == 0;
}

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
void CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
#if 1
	if (hri_can_get_TXFQS_TFQF_bit(dev.hw))
	{
		return ERR_NO_RESOURCE;
	}

	volatile _can_tx_fifo_entry *f = nullptr;
	hri_can_txfqs_reg_t put_index = hri_can_read_TXFQS_TFQPI_bf(dev.hw);

#ifdef CONF_CAN0_ENABLED
	if (dev.hw == CAN0)
	{
		f = (_can_tx_fifo_entry *)(can0_tx_fifo + put_index * CONF_CAN0_TBDS);
	}
#endif
#ifdef CONF_CAN1_ENABLED
	if (dev.hw == CAN1)
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

	f->T1.bit.FDF = hri_can_get_CCCR_FDOE_bit(dev.hw);
	f->T1.bit.BRS = hri_can_get_CCCR_BRSE_bit(dev.hw);

	hri_can_write_TXBAR_reg(dev.hw, 1 << put_index);

#else
	qq;
#endif
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	const uint32_t timeStarted = millis();
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		if (hw->RXF0S.bit.F0FL == 0)
		{

		}
	case RxBufferNumber::fifo1:
		return hw->RXF1S.bit.F1FL != 0;
	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return (hw->NDAT1.reg & (1ul << (unsigned int)whichBuffer)) != 0;
	}
	if (!hri_can_read_RXF0S_F0FL_bf(dev.hw))
	{
		return ERR_NOT_FOUND;
	}

	hri_can_rxf0s_reg_t get_index = hri_can_read_RXF0S_F0GI_bf(dev.hw);
	volatile _can_rx_fifo_entry *f = nullptr;

#ifdef CONF_CAN0_ENABLED
	if (dev.hw == CAN0)
	{
		f = (_can_rx_fifo_entry *)(can0_rx_fifo + get_index * CONF_CAN0_F0DS);
	}
#endif
#ifdef CONF_CAN1_ENABLED
	if (dev.hw == CAN1)
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

	hri_can_write_RXF0A_F0AI_bf(dev.hw, get_index);

	return ERR_NONE;
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return hw->RXF0S.bit.F0FL != 0;
	case RxBufferNumber::fifo1:
		return hw->RXF1S.bit.F1FL != 0;
	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return (hw->NDAT1.reg & ((uint32_t)1 << ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0))) != 0;
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	CanStandardMessageFilterElement::S0Type s0;
	s0.val = 0;										// disable filter, clear reserved fields
	if (mask != 0)
	{
		s0.bit.SFID1 = id;
		s0.bit.SFT = 0x02;							// classic filter
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			s0.bit.SFEC = 0x01;						// store in FIFO 0
			s0.bit.SFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			s0.bit.SFEC = 0x02;						// store in FIFO 1
			s0.bit.SFID2 = mask;
			break;
		default:
			s0.bit.SFEC = 0x07;						// store in buffer
			s0.bit.SFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			break;
		}
	}
	context->rxStdFilter[index].S0.val = s0.val;
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	volatile CanExtendedMessageFilterElement& efp = context->rxExtFilter[index];
	efp.F0.val = 0;									// disable filter
	if (mask != 0)
	{
		CanExtendedMessageFilterElement::F0Type f0;
		CanExtendedMessageFilterElement::F1Type f1;
		f0.val = 0;									// clear all fields
		f1.val = 0;									// clear all fields
		f1.bit.EFT  = 0x02;							// classic filter
		f0.bit.EFID1 = id;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			f0.bit.EFEC = 0x01;						// store in FIFO 0
			f1.bit.EFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			f0.bit.EFEC = 0x02;						// store in FIFO 1
			f1.bit.EFID2 = mask;
			break;
		default:
			f0.bit.EFEC = 0x07;
			f1.bit.EFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			break;
		}

		efp.F1.val = f1.val;						// update second word first while the filter is disabled
		efp.F0.val = f0.val;						// update first word and enable filter
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) noexcept
{
	const uint32_t nbtp = hw->NBTP.reg;
	const uint32_t tseg1 = (nbtp & CAN_NBTP_NTSEG1_Msk) >> CAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (nbtp & CAN_NBTP_NTSEG2_Msk) >> CAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (nbtp & CAN_NBTP_NSJW_Msk) >> CAN_NBTP_NSJW_Pos;
	const uint32_t brp = (nbtp & CAN_NBTP_NBRP_Msk) >> CAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
	hw->CCCR.reg |= CAN_CCCR_CCE | CAN_CCCR_INIT;
	UpdateLocalCanTiming(timing);
	hw->CCCR.reg &= ~CAN_CCCR_CCE;
}

void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
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

	hw->NBTP.reg = ((tseg1 - 1) << CAN_NBTP_NTSEG1_Pos)
				| ((tseg2 - 1) << CAN_NBTP_NTSEG2_Pos)
				| ((jumpWidth - 1) << CAN_NBTP_NSJW_Pos)
				| ((prescaler - 1) << CAN_NBTP_NBRP_Pos);
	// The fast data rate defaults to the same timing
	hw->DBTP.reg = ((tseg1 - 1) << CAN_DBTP_DTSEG1_Pos)
				| ((tseg2 - 1) << CAN_DBTP_DTSEG2_Pos)
				| ((jumpWidth - 1) << CAN_DBTP_DSJW_Pos)
				| ((prescaler - 1) << CAN_DBTP_DBRP_Pos);
}

void CanDevice::Interrupt() noexcept
{
	uint32_t ir;
	while (((ir = hw->IR.reg) & (CAN_IR_RF0N | CAN_IR_RF1N | CAN_IR_DRX | CAN_IR_TC | CAN_IR_BO | CAN_IR_RF0L | CAN_IR_RF1L)) != 0)
	{
		hw->IR.reg = ir;

		if (ir & CAN_IR_RF0N)
		{
			constexpr unsigned int waitingIndex = (unsigned int)RxBufferNumber::fifo0;
			TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex]);
			rxBuffersWaiting &= ~((uint32_t)1 << waitingIndex);
		}

		if (ir & CAN_IR_RF1N)
		{
			constexpr unsigned int waitingIndex = (unsigned int)RxBufferNumber::fifo1;
			TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex]);
			rxBuffersWaiting &= ~((uint32_t)1 << waitingIndex);
		}

		if (ir & CAN_IR_DRX)
		{
			// Check which receive buffers have new messages
			if (NumRxBuffers[whichCan] != 0)		// needed to avoid a compiler warning
			{
				uint32_t newData;
				while ((newData = hw->NDAT1.reg & rxBuffersWaiting) != 0)
				{
					const unsigned int waitingIndex = LowestSetBit(newData) + (unsigned int)RxBufferNumber::buffer0;
					TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex]);
					rxBuffersWaiting &= ~((uint32_t)1 << waitingIndex);
				}
			}
		}

		if (ir & CAN_IR_TC)
		{
			// Check which transmit buffers have finished transmitting
			if (NumTxBuffers[whichCan] != 0)		// needed to avoid a compiler warning
			{
				uint32_t transmitDone;
				while ((transmitDone = (~hw->TXBRP.reg) & (txBuffersWaiting >> 1)) != 0)
				{
					const unsigned int waitingIndex = LowestSetBit(transmitDone) + (unsigned int)TxBufferNumber::buffer0;
					TaskBase::GiveFromISR(txTaskWaiting[waitingIndex]);
					txBuffersWaiting &= ~((uint32_t)1 << waitingIndex);
				}
			}

			// Check the tx FIFO
			if ((txBuffersWaiting & 1u) != 0 && hw->TXFQS.bit.TFFL != 0)
			{
				constexpr unsigned int waitingIndex = (unsigned int)TxBufferNumber::fifo;
				TaskBase::GiveFromISR(txTaskWaiting[waitingIndex]);
				txBuffersWaiting &= ~((uint32_t)1 << waitingIndex);
			}
		}

		if (ir & CAN_IR_BO)
		{
			Disable();
			++busOffCount;
			DoHardwareInit();
			Enable();
		}

		if (ir & (CAN_IR_RF0L || CAN_IR_RF1L))
		{
			++messagesLost;
		}
	}
}

// Interrupt handlers
void CAN0_Handler() noexcept
{
	devicesByPort[0]->Interrupt();
}

void CAN1_Handler() noexcept
{
	devicesByPort[1]->Interrupt();
}

#endif
