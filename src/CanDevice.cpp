/*
 * CanDevice.cpp
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#include "CanDevice.h"

#if SUPPORT_CAN && !RP2040

#define USE_TRANSCEIVER_COMPENSATION	(1)

#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <CoreNotifyIndices.h>
#include <cstring>

#if SAME5x
# include <hri_gclk_e54.h>
#elif SAME70
# include <pmc/pmc.h>
// The following definitions are missing from the MCAN peripheral definition in ASF3
# define MCAN_RXF0C_F0OM_Pos	(31)								/**< (MCAN_RXF0C) FIFO 0 Operation Mode Position */
# define MCAN_RXF1C_F1OM_Pos	(31)								/**< (MCAN_RXF1C) FIFO 1 Operation Mode Position */
# define MCAN_TXBC_TFQM_Pos		(30)								/**< (MCAN_TXBC) Tx FIFO/Queue Mode Position */
# define MCAN_TXBC_TFQM_Msk		(0x1u << MCAN_TXBC_TFQM_Pos)		/**< (MCAN_TXBC) Tx FIFO/Queue Mode Mask */
# define MCAN_TXFQS_TFQF_Pos	(21)								/**< (MCAN_TXFQS) Tx FIFO/Queue Full Position */
# define MCAN_TXFQS_TFQF_Msk	(0x1u << MCAN_TXFQS_TFQF_Pos)		/**< (MCAN_TXFQS) Tx FIFO/Queue Full Mask */
# define MCAN_GFC_ANFS_REJECT	(0x02u << MCAN_GFC_ANFS_Pos)
# define MCAN_GFC_ANFE_REJECT	(0x02u << MCAN_GFC_ANFE_Pos)
#elif SAMC21
# include <hri_gclk_c21.h>
#elif STM32H5
# define GFC					RXGFC
# define FDCAN_GFC_ANFS_REJECT	(0x02u << FDCAN_RXGFC_ANFS_Pos)
# define FDCAN_GFC_ANFE_REJECT	(0x02u << FDCAN_RXGFC_ANFE_Pos)
# define FDCAN_GFC_RRFS			FDCAN_RXGFC_RRFS
# define FDCAN_GFC_RRFE			FDCAN_RXGFC_RRFE
#elif STM32H7
# define FDCAN_GFC_ANFS_REJECT	(0x02u << FDCAN_GFC_ANFS_Pos)
# define FDCAN_GFC_ANFE_REJECT	(0x02u << FDCAN_GFC_ANFE_Pos)
#else
# error Unsupported processor
#endif

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

/**@}*/
/**
 * \brief CAN transmit event FIFO element.
 */
struct CanDevice::TxEvent
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
			uint32_t TXTS : 16; /*!< Tx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t ET : 2;    /*!< Event type */
			uint32_t MM : 8;    /*!< Message marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;
};

// Macros to handle the differing naming of registers and fields between the SAME70 and the SAME5x/C21
#if SAME5x || SAMC21

# define REG(_x)					_x.reg
# define CAN_(_x)					CAN_ ## _x
# define CAN_Val(_x,_y)				CAN_ ## _x(_y)
# define READBITS(_hw,_x,_y)		(hw)->_x.bit._y
# define WRITEBITS(_hw,_x,_y,_val)	(hw)->_x.bit._y = _val

#elif SAME70

# define REG(_x)					MCAN_ ## _x
# define CAN_(_x)					MCAN_ ## _x
# define CAN_Val(_x,_y)				MCAN_ ## _x(_y)
# define READBITS(_hw,_x,_y)		((((hw)->MCAN_ ## _x) & (MCAN_ ## _x ## _ ## _y ## _Msk)) >> (MCAN_ ## _x ## _ ## _y ## _Pos))
# define WRITEBITS(_hw,_x,_y,_val)	(hw)->MCAN_ ## _x = (((hw)->MCAN_ ## _x) & ~(MCAN_ ## _x ## _ ## _y ## _Msk)) | ((_val << (MCAN_ ## _x ## _ ## _y ## _Pos)) & (MCAN_ ## _x ## _ ## _y ## _Msk))

# define CAN0						MCAN0
# define CAN1						MCAN1
# define CAN0_IRQn					MCAN0_INT0_IRQn
# define CAN1_IRQn					MCAN1_INT0_IRQn
# define CAN0_Handler				MCAN0_INT0_Handler
# define CAN1_Handler				MCAN1_INT0_Handler

#elif STM32

// The STM32H5 uses a similar Bosch device as the Microchip MCUs with the following differences:
// - Some of the register offsets are different
// - There is no MRCFG (message RAM configuration) register because it has 10Kb of dedicated RAM (shared between all CAN peripherals)
// - Register GFC is renamed RXGFC on the STM32H5
// - There are no SIDFC, XIDFC, NDAT1, NAT2, RXF0C, RXBC, RXF1C, RXESC, TXESC, TXEFC registers on the STM32H5
// - The TXBC register contains only one bit, TFQM
// - The IR register has no RF0W, RF1W, TWFW, DRX, BEC, or BEU bits
// - Dedicated Rx buffers are not supported
// - The two receive FIFOs are fixed size, 3 elements each
// - The number of transmit buffers is fixed at 3. The set of 3 buffers may be configured to be a FIFO or a queue. There are no dedicated transmit buffers.
//   Only by configuring it as a queue can we ensure that high priority messages are sent first.
# define REG(_x)					_x
# define CAN_(_x)					FDCAN_ ## _x
# define CAN_Val(_x,_y)				((_y) << FDCAN_ ## _x ## _Pos)
# define READBITS(_hw,_x,_y)		((((hw)->_x) & (FDCAN_ ## _x ## _ ## _y ## _Msk)) >> (FDCAN_ ## _x ## _ ## _y ## _Pos))
# define WRITEBITS(_hw,_x,_y,_val)	(hw)->_x = (((hw)->_x) & ~(FDCAN_ ## _x ## _ ## _y ## _Msk)) | ((_val << (FDCAN_ ## _x ## _ ## _y ## _Pos)) & (FDCAN_ ## _x ## _ ## _y ## _Msk))

# if STM32H5
#  define CAN0						FDCAN1_NS
#  define CAN1						FDCAN2_NS
# else
#  define CAN0						FDCAN1
#  define CAN1						FDCAN2
# endif

# define CAN0_IRQn					FDCAN1_IT0_IRQn
# define CAN1_IRQn					FDCAN2_IT0_IRQn
# define CAN0_Handler				FDCAN1_IT0_Handler
# define CAN1_Handler				FDCAN2_IT0_Handler

#endif

static Can * const CanPorts[2] = { CAN0, CAN1 };
static const IRQn IRQnsByPort[2] = { CAN0_IRQn, CAN1_IRQn };
static CanDevice *devicesByPort[2] = { nullptr, nullptr };

static CanDevice devices[NumCanDevices];

inline uint32_t CanDevice::GetRxBufferSize() const noexcept { return sizeof(CanRxBufferHeader)/sizeof(uint32_t) + (config->dataSize >> 2); }
inline uint32_t CanDevice::GetTxBufferSize() const noexcept { return sizeof(CanTxBufferHeader)/sizeof(uint32_t) + (config->dataSize >> 2); }
inline CanRxBufferHeader *CanDevice::GetRxFifo0Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx0Fifo + (index * GetRxBufferSize())); }
inline CanRxBufferHeader *CanDevice::GetRxFifo1Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx1Fifo + (index * GetRxBufferSize())); }
inline CanRxBufferHeader *CanDevice::GetRxBuffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rxBuffers + (index * GetRxBufferSize())); }
inline CanTxBufferHeader *CanDevice::GetTxBuffer(uint32_t index) const noexcept { return (CanTxBufferHeader*)(txBuffers + (index * GetTxBufferSize())); }
inline volatile CanDevice::TxEvent *CanDevice::GetTxEvent(uint32_t index) const noexcept { return &txEventFifo[index]; }

// Clear statistics
void CanDevice::CanStats::Clear() noexcept
{
	messagesQueuedForSending = messagesReceived = messagesLost = protocolErrors = busOffCount = 0;
}

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	if (   p_whichCan >= NumCanDevices									// device number out of range
		|| p_whichPort >= 2												// CAN instance number out of range
		|| devicesByPort[p_whichPort] != nullptr						// CAN instance number already in use
	   )
	{
		return nullptr;
	}

	CanDevice& dev = devices[p_whichCan];
	if (dev.hw != nullptr)												// device instance already in use
	{
		return nullptr;
	}

#if STM32H5
	// STM32H5 supports exactly 3 buffers in each FIFO and no dedicated transmit or receive buffers
	if (   p_config.numRxBuffers != 0 || p_config.numTxBuffers != 0 || p_config.rxFifo0Size != 3
		|| p_config.rxFifo1Size != 3 || p_config.txFifoSize != 3 || p_config.txEventFifoSize != 3
		|| p_config.numShortFilterElements != 28 || p_config.numExtendedFilterElements != 8
	   )
	{
		return nullptr;
	}
#endif

	// Set up device number, peripheral number, hardware address etc.
	dev.whichCan = p_whichCan;
	dev.whichPort = p_whichPort;
	dev.hw = CanPorts[p_whichPort];
	dev.config = &p_config;
	dev.txCallback = p_txCallback;
	devicesByPort[p_whichPort] = &dev;

	// Set up pointers to the individual parts of the buffer memory
	memset(memStart, 0, p_config.GetMemorySize() * sizeof(uint32_t));	// clear out filters, transmit pending flags etc.

#if SAME70
	// Set upper 16 bits of DMA addresses. The CAN memory must not cross a 64kb boundary.
	if (dev.whichPort == 0)
	{
		MATRIX->CCFG_CAN0 = (MATRIX->CCFG_CAN0 & 0x0000FFFF) | (reinterpret_cast<uint32_t>(memStart) & 0xFFFF0000);
	}
	else
	{
		MATRIX->CCFG_SYSIO = (MATRIX->CCFG_SYSIO & 0x0000FFFF) | (reinterpret_cast<uint32_t>(memStart) & 0xFFFF0000);
	}
#endif

	dev.rxStdFilter = (CanStandardMessageFilterElement*)memStart;
	memStart += p_config.GetStandardFiltersMemSize();
	dev.rxExtFilter = (CanExtendedMessageFilterElement*)memStart;
	memStart += p_config.GetExtendedFiltersMemSize();
	dev.rx0Fifo = memStart;
	memStart += p_config.rxFifo0Size * p_config.GetRxBufferSize();
	dev.rx1Fifo = memStart;
	memStart += p_config.rxFifo1Size * p_config.GetRxBufferSize();
	dev.rxBuffers = memStart;
	memStart += p_config.numRxBuffers * p_config.GetRxBufferSize();
	dev.txEventFifo = (TxEvent*)memStart;
	memStart += p_config.GetTxEventFifoMemSize();
	dev.txBuffers = memStart;

	dev.useFDMode = (p_config.dataSize > 8);							// assume we want standard CAN if the max data size is 8
	dev.stats.Clear();
#ifdef RTOS
	for (volatile TaskHandle& h : dev.txTaskWaiting) { h = nullptr; }
	for (volatile TaskHandle& h : dev.rxTaskWaiting) { h = nullptr; }
	dev.rxBuffersWaiting = 0;
#endif

	dev.UpdateLocalCanTiming(timing);									// sets NBTP, DBTP and usingBrs

	// Enable the clock
#if SAME5x || SAMC21
	if (p_whichPort == 0)
	{
		MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN0;
		hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, GclkNum48MHz | GCLK_PCHCTRL_CHEN);
	}
	else
	{
		MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN1;
		hri_gclk_write_PCHCTRL_reg(GCLK, CAN1_GCLK_ID, GclkNum48MHz | GCLK_PCHCTRL_CHEN);
	}
#elif SAME70
	pmc_disable_pck(PMC_PCK_5);
	pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(9));				// run PCLK5 at 48MHz
	pmc_enable_pck(PMC_PCK_5);

	if (p_whichPort == 0)
	{
		pmc_enable_periph_clk(ID_MCAN0);
	}
	else
	{
		pmc_enable_periph_clk(ID_MCAN1);
	}
#elif STM32
	// Enable APB clock
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
	(void)RCC->APB1HENR;												// delay after an RCC peripheral clock enabling
	// Enable 48MHz CAN clock
# if STM32H5
	MODIFY_REG(RCC->CCIPR5, RCC_CCIPR5_FDCANSEL, LL_RCC_FDCAN_CLKSOURCE_PLL1Q);
# elif STM32H7
	//TODO
# else
#  error Unsupported MCU
# endif
#endif

	dev.DoHardwareInit();
	return &dev;
}

// get bits 2..15 of an address
static inline uint32_t Bits2to15(const volatile void *addr) noexcept
{
	return reinterpret_cast<uint32_t>(addr) & 0x0000FFFC;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	Disable();

	if (useFDMode)
	{
		hw->REG(CCCR) |= CAN_(CCCR_FDOE) | CAN_(CCCR_BRSE);
	}
	else
	{
		hw->REG(CCCR) &= ~(CAN_(CCCR_FDOE) | CAN_(CCCR_BRSE));
	}

	hw->REG(CCCR) |= CAN_(CCCR_TXP);										// enable transmit pause

#if SAME5x || SAMC21
	hw->MRCFG.reg = CAN_MRCFG_QOS_MEDIUM;
#endif
	hw->REG(NBTP) = nbtp;
	hw->REG(DBTP) = dbtp;
#if USE_TRANSCEIVER_COMPENSATION
	const uint32_t dTseg1 = ((dbtp & CAN_(DBTP_DTSEG1_Msk)) >> CAN_(DBTP_DTSEG1_Pos)) + 1;
	hw->REG(TDCR) = CAN_Val(TDCR_TDCO, dTseg1) | CAN_Val(TDCR_TDCF, dTseg1);
#else
	hw->REG(TDCR) = 0;														// use just the measured transceiver delay
#endif

#if STM32H5
	hw->REG(TXBC) = (0 << CAN_(TXBC_TFQM_Pos));								// configure transmit buffers: FIFO not queue
#else
	hw->REG(RXF0C) = 														// configure receive FIFO 0
		  (0 << CAN_(RXF0C_F0OM_Pos))										// blocking mode not overwrite mode
		| CAN_Val(RXF0C_F0WM, 0)											// no watermark interrupt
		| CAN_Val(RXF0C_F0S, config->rxFifo0Size)							// number of entries
		| Bits2to15(rx0Fifo);												// address - don't use CAN_(RXF0C_F0SA) here, it is defined strangely on the SAME70
	hw->REG(RXF1C) = 														// configure receive FIFO 1
		  (0 << CAN_(RXF1C_F1OM_Pos))										// blocking mode not overwrite mode
		| CAN_Val(RXF1C_F1WM, 0)											// no watermark interrupt
		| CAN_Val(RXF1C_F1S, config->rxFifo1Size)							// number of entries
		| Bits2to15(rx1Fifo);												// address - don't use CAN_(RXF0C_F1SA) here, it is defined strangely on the SAME70
	hw->REG(RXBC) = Bits2to15(rxBuffers);									// dedicated buffers start address - don't use CAN_(RXBC_RBSA) here, it is defined strangely on the SAME70

	const uint32_t dataSizeCode = (config->dataSize <= 24) ? (config->dataSize >> 2) - 2 : (config->dataSize >> 4) + 3;
	hw->REG(RXESC) = CAN_Val(RXESC_F0DS, dataSizeCode)						// receive fifo 0 data size
					| CAN_Val(RXESC_F1DS, dataSizeCode)						// receive fifo 1 data size
					| CAN_Val(RXESC_RBDS, dataSizeCode);					// receive buffer data size
	hw->REG(TXESC) = CAN_Val(TXESC_TBDS, dataSizeCode);						// transmit buffer data size
	hw->REG(TXBC) = 														// configure transmit buffers
		  (0 << CAN_(TXBC_TFQM_Pos))										// FIFO not queue
		| CAN_Val(TXBC_TFQS, config->txFifoSize)							// number of Tx fifo entries
		| CAN_Val(TXBC_NDTB, config->numTxBuffers)							// number of dedicated Tx buffers
		| Bits2to15(txBuffers);												// address - don't use CAN_(TXBC_TBSA) here, it is defined strangely on the SAME70
	hw->REG(TXEFC) =  														// configure Tx event fifo
		  CAN_Val(TXEFC_EFWM, 0)											// no watermark interrupt
		| CAN_Val(TXEFC_EFS, config->txEventFifoSize)						// event FIFO size
		| Bits2to15(txEventFifo);											// address - don't use CAN_(TXEFC_EFSA) here, it is defined strangely on the SAME70
#endif
	hw->REG(GFC) =
		  CAN_(GFC_ANFS_REJECT)
		| CAN_(GFC_ANFE_REJECT)
		| CAN_(GFC_RRFS)
		| CAN_(GFC_RRFE);
#if !STM32H5
	hw->REG(SIDFC) = CAN_Val(SIDFC_LSS, config->numShortFilterElements)		// number of short filter elements
					| Bits2to15(rxStdFilter);								// short filter start address - don't use CAN_(SIDFC_FLSSA) here, it is defined strangely on the SAME70
	hw->REG(XIDFC) = CAN_Val(XIDFC_LSE, config->numExtendedFilterElements)		// number of extended filter elements
					| Bits2to15(rxExtFilter);								// extended filter start address - don't use CAN_(SIDFC_FLESA) here, it is defined strangely on the SAME70
#endif
	hw->REG(XIDAM) = 0x1FFFFFFF;

	hw->REG(IR) = 0xFFFFFFFF;												// clear all interrupt sources

	// Set up the timestamp counter
#if SAME70
	// Use external timestamp counter, which is TC0. So TC0 must be the step clock lower 16 bits on SAME70 boards.
	hw->MCAN_TSCC = MCAN_TSCC_TSS_EXT_TIMESTAMP;							// select external timestamp counter
#elif STM32
	// Use external timestamp counter, which is timer 3.
	hw->TSCC = 0x02;														// select external timestamp counter
#else
	hw->TSCC.reg = CAN_TSCC_TSS_INC | CAN_TSCC_TCP(0);						// run timestamp counter at CAN bit speed (prescaler = 1)
#endif

#ifdef RTOS
	const IRQn irqn = IRQnsByPort[whichPort];
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);

	hw->REG(ILS) = 0;														// all interrupt sources assigned to interrupt line 0 for now
	statusMask =          CAN_(IR_RF0N)  | CAN_(IR_RF1N)
#if !STM32H5
						| CAN_(IR_DRX)
#endif
						| CAN_(IR_TC)  | CAN_(IR_BO)  | CAN_(IR_RF0L)  | CAN_(IR_RF1L)  | CAN_(IR_PED)  | CAN_(IR_PEA);
	uint32_t intEnable =  CAN_(IE_RF0NE) | CAN_(IE_RF1NE)
#if !STM32H5
						| CAN_(IE_DRXE)
#endif
						| CAN_(IE_TCE) | CAN_(IE_BOE) | CAN_(IE_RF0LE) | CAN_(IE_RF1LE) | CAN_(IE_PEDE) | CAN_(IE_PEAE);
	if (txCallback != nullptr)
	{
		intEnable |= CAN_(IE_TEFNE);
		statusMask |= CAN_(IR_TEFN);
	}
	hw->REG(IE) = intEnable;												// enable the interrupt sources that we want
	hw->REG(ILE) = CAN_(ILE_EINT0);											// enable interrupt line 0

	NVIC_EnableIRQ(irqn);
#else
	hw->REG(IE) = 0;														// disable all interrupt sources
	hw->REG(ILE) = 0;
#endif
	// Leave the device disabled. Client must call Enable() to enable it after setting up the receive filters.
}

// Set the extended ID mask. May only be used while the interface is disabled.
void CanDevice::SetExtendedIdMask(uint32_t mask) noexcept
{
	hw->REG(XIDAM) = mask;
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (hw != nullptr)
	{
		NVIC_DisableIRQ(IRQnsByPort[whichPort]);
		Disable();
		devicesByPort[whichPort] = nullptr;									// free the port
		hw = nullptr;														// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	hw->REG(CCCR) &= ~CAN_(CCCR_INIT);
	while ((hw->REG(CCCR) & CAN_(CCCR_INIT)) != 0) { }
}

// Disable this device
void CanDevice::Disable() noexcept
{
	hw->REG(CCCR) |= CAN_(CCCR_INIT);
	while ((hw->REG(CCCR) & CAN_(CCCR_INIT)) == 0) { }
	hw->REG(CCCR) |= CAN_(CCCR_CCE);
}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	uint32_t txefs;
	while (((txefs = hw->REG(TXEFS)) & CAN_(TXEFS_EFFL_Msk)) != 0)
	{
		const uint32_t index = (txefs & CAN_(TXEFS_EFGI_Msk)) >> CAN_(TXEFS_EFGI_Pos);
		const volatile TxEvent *const elemPtr = GetTxEvent(index);
		Cache::InvalidateAfterDMAReceive(elemPtr, sizeof(TxEvent));		// this is essential to avoid reading old events
		TxEvent event;
		event.R0.val = elemPtr->R0.val;
		event.R1.val = elemPtr->R1.val;
		hw->REG(TXEFA) = index;											// doing this early instead of at the end of the loop helps
		if (event.R1.bit.ET == 1)
		{
			CanId id;
			id.SetReceivedId(event.R0.bit.ID);
			p_txCallback(event.R1.bit.MM, id, event.R1.bit.TXTS);
		}
		__DSB();														// probably not needed, but just in case
	}
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return hw->REG(ECR);
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	if (whichBuffer == TxBufferNumber::fifo)
	{
#ifdef RTOS
		bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
		if (!bufferFree && timeout != 0)
		{
			const unsigned int bufferIndex = READBITS(hw, TXFQS, TFQPI);
			const uint32_t trigMask = (uint32_t)1 << bufferIndex;

			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();

			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) |= trigMask;
			}

			bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
			// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout);
				bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;

			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) &= ~trigMask;
			}
		}
#else
		do
		{
			bufferFree = READBITS(hw, TXFQS, TFQF) == 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	else
	{
		const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
#ifdef RTOS
		bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
		if (!bufferFree && timeout != 0)
		{
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) |= trigMask;
			}
			bufferFree = (hw->REG(TXBRP) & trigMask) == 0;

			// In the following, when we call TaskBase::Take() assume that the task may get woken up early
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout);
				bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) &= ~trigMask;
			}
		}
#else
		do
		{
			bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	return bufferFree;
}

#if 0	// not currently used

// Return the number of messages waiting to be sent in the transmit FIFO
unsigned int CanDevice::NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept
{
	if (whichBuffer == TxBufferNumber::fifo)
	{
		return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
	}

	const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
	return (hw->REG(TXBRP) >> bufferIndex) & 1u;
}

#endif

void CanDevice::CopyMessageForTransmit(CanMessageBuffer *buffer, volatile CanTxBufferHeader *f) noexcept
{
	if (buffer->extId)
	{
		f->T0.val = buffer->id.GetWholeId();
		f->T0.bit.XTD = 1;
	}
	else
	{
		f->T0.val = buffer->id.GetWholeId() << 18;			// an 11-bit identifier is stored into ID[28:18]
		f->T0.bit.XTD = 0;
	}

	f->T0.bit.RTR = buffer->remote;
	f->T1.bit.MM = buffer->marker;
	f->T1.bit.EFCbit = buffer->reportInFifo;
	uint32_t dataLength = buffer->dataLength;
	volatile uint32_t *dataPtr = f->GetDataPointer();
	if (dataLength <= 8)
	{
		f->T1.bit.DLC = dataLength;
		dataPtr[0] = buffer->msg.raw32[0];
		dataPtr[1] = buffer->msg.raw32[1];
	}
	else
	{
		while (dataLength & 3)
		{
			buffer->msg.raw[dataLength++] = 0;				// pad length to a multiple of 4 bytes, setting any additional bytes we send to zero in case the message ends with a string
		}

		if (dataLength <= 24)
		{
			// DLC values 9, 10, 11, 12 code for lengths 12, 16, 20, 24
			uint8_t dlc = (dataLength >> 2) + 6;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 6);								// copy 3, 4, 5 or 6 words
		}
		else
		{
			// DLC values 13, 14, 15 code for lengths 32, 48, 64
			while (dataLength & 12)
			{
				buffer->msg.raw32[dataLength >> 2] = 0;		// pad length to a multiple of 16 bytes, setting any additional bytes we send to zero in case the message ends with a string
				dataLength += 4;
			}

			uint8_t dlc = (dataLength >> 4) + 11;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 11);
		}
	}

	f->T1.bit.FDF = buffer->fdMode;
	f->T1.bit.BRS = usingBrs && buffer->useBrs;

	++stats.messagesQueuedForSending;
}

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// If the buffer or fifo is used by multiple tasks then the caller must acquire the mutex for that buffer or fifo first
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
	if ((uint32_t)whichBuffer < (uint32_t)TxBufferNumber::buffer0 + config->numTxBuffers)
	{
		const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
		const uint32_t bufferIndex = (whichBuffer == TxBufferNumber::fifo)
										? READBITS(hw, TXFQS, TFQPI)
											: (uint32_t)whichBuffer - (uint32_t)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
		if (!bufferFree)
		{
			// Retrieve details of the packet we are about to cancel
			cancelledId = GetTxBuffer(bufferIndex)->T0.bit.ID;
			// Cancel transmission of the oldest packet
			hw->REG(TXBCR) = trigMask;
			do
			{
				delay(1);
			}
			while ((hw->REG(TXBRP) & trigMask) != 0 || (whichBuffer == TxBufferNumber::fifo && READBITS(hw, TXFQS, TFQF)));
		}

		CopyMessageForTransmit(buffer, GetTxBuffer(bufferIndex));
		__DSB();								// this is needed on the SAME70, otherwise incorrect data sometimes gets transmitted
		hw->REG(TXBAR) = trigMask;
	}
	return cancelledId;
}

void CanDevice::CopyReceivedMessage(CanMessageBuffer *null buffer, const volatile CanRxBufferHeader *f) noexcept
{
	// The CAN has written the message directly to memory, so we must invalidate the cache before we read it
	Cache::InvalidateAfterDMAReceive(f, sizeof(CanRxBufferHeader) + 64);						// flush the header and up to 64 bytes of data

	if (buffer != nullptr)
	{
		buffer->extId = f->R0.bit.XTD;
		buffer->id.SetReceivedId((buffer->extId) ? f->R0.bit.ID : f->R0.bit.ID >> 18);			// a standard identifier is stored into ID[28:18]
		buffer->remote = f->R0.bit.RTR;
		buffer->useBrs = f->R1.bit.BRS;
		buffer->timeStamp = f->R1.bit.RXTS;
		const uint8_t dlc = f->R1.bit.DLC;
		const volatile uint32_t *data = f->GetDataPointer();
		static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

		switch (dlc)
		{
		case 5:
		case 6:
		case 7:
		case 8:
			buffer->msg.raw32[1] = data[1];
			[[fallthrough]];
		case 1:
		case 2:
		case 3:
		case 4:
			buffer->msg.raw32[0] = data[0];
			[[fallthrough]];
		case 0:
			buffer->dataLength = dlc;
			break;

		case 15:		// 64 bytes
			buffer->msg.raw32[12] = data[12];
			buffer->msg.raw32[13] = data[13];
			buffer->msg.raw32[14] = data[14];
			buffer->msg.raw32[15] = data[15];
			[[fallthrough]];
		case 14:		// 48 bytes
			buffer->msg.raw32[8] = data[8];
			buffer->msg.raw32[9] = data[9];
			buffer->msg.raw32[10] = data[10];
			buffer->msg.raw32[11] = data[11];
			[[fallthrough]];
		case 13:		// 32 bytes
			buffer->msg.raw32[6] = data[6];
			buffer->msg.raw32[7] = data[7];
			[[fallthrough]];
		case 12:		// 24 bytes
			buffer->msg.raw32[5] = data[5];
			[[fallthrough]];
		case 11:		// 20 bytes
			buffer->msg.raw32[4] = data[4];
			[[fallthrough]];
		case 10:		// 16 bytes
			buffer->msg.raw32[3] = data[3];
			[[fallthrough]];
		case 9:			// 12 bytes
			buffer->msg.raw32[0] = data[0];
			buffer->msg.raw32[1] = data[1];
			buffer->msg.raw32[2] = data[2];
			buffer->dataLength = dlc2len[dlc];
		}
	}

	++stats.messagesReceived;
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		{
			// Check for a received message and wait if necessary
#ifdef RTOS
			if (READBITS(hw, RXF0S, F0FL) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (READBITS(hw, RXF0S, F0FL) != 0) || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), READBITS(hw, RXF0S, F0FL) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (READBITS(hw, RXF0S, F0FL) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			const uint32_t getIndex = READBITS(hw, RXF0S, F0GI);
			CopyReceivedMessage(buffer, GetRxFifo0Buffer(getIndex));

			// Tell the hardware that we have taken the message
			WRITEBITS(hw, RXF0A, F0AI, getIndex);
		}
		return true;

	case RxBufferNumber::fifo1:
		// Check for a received message and wait if necessary
		{
#ifdef RTOS
			if (READBITS(hw, RXF1S, F1FL) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (READBITS(hw, RXF1S, F1FL) != 0) || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), READBITS(hw, RXF1S, F1FL) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (READBITS(hw, RXF1S, F1FL) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			const uint32_t getIndex = READBITS(hw, RXF1S, F1GI);
			CopyReceivedMessage(buffer, GetRxFifo1Buffer(getIndex));

			// Tell the hardware that we have taken the message
			WRITEBITS(hw, RXF1A, F1AI, getIndex);
		}
		return true;

	default:
#if !STM32H5
		if ((uint32_t)whichBuffer < (uint32_t)RxBufferNumber::buffer0 + config->numRxBuffers)
		{
			// Check for a received message and wait if necessary
			// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
			const uint32_t bufferNumber = (unsigned int)whichBuffer - (unsigned int)RxBufferNumber::buffer0;
			const uint32_t ndatMask = (uint32_t)1 << bufferNumber;
# ifdef RTOS
			if ((hw->REG(NDAT1) & ndatMask) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				rxBuffersWaiting |= ndatMask;
				const bool success = (hw->REG(NDAT1) & ndatMask) != 0 || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), (hw->REG(NDAT1) & ndatMask) != 0);
				rxBuffersWaiting &= ~ndatMask;
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
# else
			while ((hw->REG(NDAT1) & ndatMask) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
# endif
			// Process the received message into the buffer
			CopyReceivedMessage(buffer, GetRxBuffer(bufferNumber));

			// Tell the hardware that we have taken the message
			hw->REG(NDAT1) = ndatMask;
			return true;
		}
#endif
		return false;
	}
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return READBITS(hw, RXF0S, F0FL) != 0;
	case RxBufferNumber::fifo1:
		return READBITS(hw, RXF1S, F1FL) != 0;
	default:
#if STM32H5
		return false;
#else
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return (hw->REG(NDAT1) & ((uint32_t)1 << ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0))) != 0;
#endif
	}
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
		rxStdFilter[index].S0.val = 0;
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
		CanStandardMessageFilterElement::S0Type s0;
		s0.val = 0;										// disable filter, clear reserved fields
		s0.bit.SFID1 = id;
		s0.bit.SFT = 0x02;								// classic filter
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			s0.bit.SFEC = 0x01;							// store in FIFO 0
			s0.bit.SFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			s0.bit.SFEC = 0x02;							// store in FIFO 1
			s0.bit.SFID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				s0.bit.SFEC = 0x07;						// store in buffer
				s0.bit.SFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				s0.bit.SFEC = 0x00;						// discard message
				s0.bit.SFID2 = mask;
			}
			break;
		}
		rxStdFilter[index].S0.val = s0.val;
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		rxExtFilter[index].F0.val = 0;					// disable filter
	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		volatile CanExtendedMessageFilterElement& efp = rxExtFilter[index];
		efp.F0.val = 0;									// disable filter

		CanExtendedMessageFilterElement::F0Type f0;
		CanExtendedMessageFilterElement::F1Type f1;
		f0.val = 0;										// clear all fields
		f1.val = 0;										// clear all fields
		f1.bit.EFT  = 0x02;								// classic filter
		f0.bit.EFID1 = id;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			f0.bit.EFEC = 0x01;							// store in FIFO 0
			f1.bit.EFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			f0.bit.EFEC = 0x02;							// store in FIFO 1
			f1.bit.EFID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				f0.bit.EFEC = 0x07;
				f1.bit.EFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				f0.bit.EFEC = 0x00;						// discard message
				f1.bit.EFID2 = mask;
			}
			break;
		}

		efp.F1.val = f1.val;							// update second word first while the filter is disabled
		efp.F0.val = f0.val;							// update first word and enable filter
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) const noexcept
{
	const uint32_t localNbtp = hw->REG(NBTP);
	const uint32_t nTseg1 = ((localNbtp & CAN_(NBTP_NTSEG1_Msk)) >> CAN_(NBTP_NTSEG1_Pos)) + 1;
	const uint32_t nTseg2 = ((localNbtp & CAN_(NBTP_NTSEG2_Msk)) >> CAN_(NBTP_NTSEG2_Pos)) + 1;
	const uint32_t nJw = ((localNbtp & CAN_(NBTP_NSJW_Msk)) >> CAN_(NBTP_NSJW_Pos)) + 1;
	const uint32_t nBrp = ((localNbtp & CAN_(NBTP_NBRP_Msk)) >> CAN_(NBTP_NBRP_Pos)) + 1;
	timing.period = (nTseg1 + nTseg2 + 1) * nBrp;
	timing.nTseg1 = nTseg1 * nBrp - 1;
	timing.nJumpWidth = nJw * nBrp;

	const uint32_t localDbtp = hw->REG(DBTP);
	const uint32_t dTseg1 = ((localDbtp & CAN_(DBTP_DTSEG1_Msk)) >> CAN_(DBTP_DTSEG1_Pos)) + 1;
	const uint32_t dTseg2 = ((localDbtp & CAN_(DBTP_DTSEG2_Msk)) >> CAN_(DBTP_DTSEG2_Pos)) + 1;
	const uint32_t dJw = ((localDbtp & CAN_(DBTP_DSJW_Msk)) >> CAN_(DBTP_DSJW_Pos)) + 1;
	const uint32_t dBrp = ((localDbtp & CAN_(DBTP_DBRP_Msk)) >> CAN_(DBTP_DBRP_Pos)) + 1;
	const uint32_t dPeriod = (dTseg1 + dTseg2 + 1) * dBrp;
	timing.dataRateMultiplier = timing.period/dPeriod - 1;
	timing.dTseg1 = dTseg1 * dBrp - 1;
	timing.dJumpWidth = dJw * dBrp;
	timing.spare1 = 0x0F;
	timing.spare2 = 0xFF;
}

// Calculate and store the nbtp and dbtp values needed to achieve the requested timing, and write them to the hardware
void CanDevice::ChangeLocalCanTiming(const CanTiming &timing) noexcept
{
	Disable();
	UpdateLocalCanTiming(timing);						// set up nbtp and dbtp variables
	hw->REG(NBTP) = nbtp;
	hw->REG(DBTP) = dbtp;
#if USE_TRANSCEIVER_COMPENSATION
	const uint32_t dTseg1 = ((dbtp & CAN_(DBTP_DTSEG1_Msk)) >> CAN_(DBTP_DTSEG1_Pos)) + 1;
	hw->REG(TDCR) = CAN_Val(TDCR_TDCO, dTseg1) | CAN_Val(TDCR_TDCF, dTseg1);
#else
	hw->REG(TDCR) = 0;														// use just the measured transceiver delay
#endif
	Enable();
}

// Calculate and store the nbtp and dbtp values needed to achieve the requested timing, but don't write them to the hardware
// Call with the CAN subsystem disabled
void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
	// Sort out the bit timing for the arbitration phase
	uint32_t nPeriod = timing.period;
	uint32_t nTseg1 = timing.nTseg1 + 1;
	uint32_t nJumpWidth = timing.nJumpWidth;
	uint32_t nPrescaler = 1;							// 48MHz main clock
	uint32_t nTseg2;

	// Use the highest prescaled clock frequency we can in order to get the most accurate timing
	for (;;)
	{
		nTseg2 = nPeriod - nTseg1 - 1;
		if (nTseg1 <= 256 && nTseg2 <= 128)
		{
			break;
		}

		// Currently we always use a prescaler that is a power of 2, but we could be more general
		nPrescaler <<= 1;
		nPeriod >>= 1;
		nTseg1 >>= 1;
		nJumpWidth >>= 1;
	}

	if (nJumpWidth > nTseg2) { nJumpWidth = nTseg2; }	// jump width cannot exceed tseg2

#if !SAME70
	bitPeriod = nPeriod * nPrescaler;					// the actual CAN normal bit period in 48MHz clocks (may be different from timing.period)
#endif

	nbtp = ((nTseg1 - 1) << CAN_(NBTP_NTSEG1_Pos))
		| ((nTseg2 - 1) << CAN_(NBTP_NTSEG2_Pos))
		| ((nJumpWidth - 1) << CAN_(NBTP_NSJW_Pos))
		| ((nPrescaler - 1) << CAN_(NBTP_NBRP_Pos));

	if (useFDMode && timing.IsUsingBrs())
	{
		uint32_t dPeriod = timing.period/(timing.dataRateMultiplier + 1);
		uint32_t dTseg1 = timing.dTseg1 + 1;
		uint32_t dJumpWidth = timing.dJumpWidth;
		uint32_t dPrescaler = 1;							// 48MHz main clock
		uint32_t dTseg2;

		// Use the highest prescaled clock frequency we can in order to get the most accurate timing
		for (;;)
		{
			dTseg2 = dPeriod - dTseg1 - 1;
			if (dTseg1 <= 32 && dTseg2 <= 16)
			{
				break;
			}

			// Currently we always use a prescaler that is a power of 2, but we could be more general
			dPrescaler <<= 1;
			dPeriod >>= 1;
			dTseg1 >>= 1;
			dJumpWidth >>= 1;
		}

		if (dJumpWidth > dTseg2) { dJumpWidth = dTseg2; }	// jump width cannot exceed tseg2
#if SAME70
		if (dJumpWidth > 8) { dJumpWidth = 8; }				// jump width cannot exceed 8 on the SAME70 (on the SAME5x it can be as large as tseg2)
#endif

		dbtp = ((dTseg1 - 1) << CAN_(DBTP_DTSEG1_Pos))
			 | ((dTseg2 - 1) << CAN_(DBTP_DTSEG2_Pos))
			 | ((dJumpWidth - 1) << CAN_(DBTP_DSJW_Pos))
			 | ((dPrescaler - 1) << CAN_(DBTP_DBRP_Pos))
#if USE_TRANSCEIVER_COMPENSATION
			 | CAN_(DBTP_TDC)
#endif
				;
		usingBrs = true;
	}
	else
	{
		// We are not using BRS. We default the fast data rate to 2Mbps (or lower if the prescaler is greater than 1) with fixed timing,
		// just to have some sensible values to write to the register.
		constexpr uint32_t fast_period = CanTiming::ClockFrequency/2'000'000;	// 2Mbps divided by the prescaler
		constexpr uint32_t fast_tseg1 = fast_period/2 - 1;						// set sample point to 50%
		constexpr uint32_t fast_tseg2 = fast_period - fast_tseg1 - 1;			// make up the correct period
		constexpr uint32_t fast_jumpWidth = fast_tseg2;							// set jump width to maximum
		dbtp = ((fast_tseg1 - 1) << CAN_(DBTP_DTSEG1_Pos))
			| ((fast_tseg2 - 1) << CAN_(DBTP_DTSEG2_Pos))
			| ((fast_jumpWidth - 1) << CAN_(DBTP_DSJW_Pos))
			| ((nPrescaler - 1) << CAN_(DBTP_DBRP_Pos));
		usingBrs = false;
	}
}

void CanDevice::GetAndClearStats(CanDevice::CanStats& dst) noexcept
{
	AtomicCriticalSectionLocker lock;
	dst = stats;
	stats.Clear();
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	uint32_t ir;
	while ((ir = hw->REG(IR) & statusMask) != 0)
	{
		hw->REG(IR) = ir;

		// Test whether messages have been received into fifo 0
		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if ((ir & CAN_(IR_RF0N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex], NotifyIndices::CanDevice);
		}

		// Test whether messages have been received into fifo 1
		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if ((ir & CAN_(IR_RF1N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex], NotifyIndices::CanDevice);
		}

#if !STM32H5
		// Test whether messages have been received into receive buffers
		if (ir & CAN_(IR_DRX))
		{
			// Check which receive buffers have new messages
			uint32_t newData;
			while ((newData = hw->REG(NDAT1) & rxBuffersWaiting) != 0)
			{
				const unsigned int rxBufferNumber = LowestSetBit(newData);
				rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
				const unsigned int waitingIndex = rxBufferNumber + (unsigned int)RxBufferNumber::buffer0;
				if (waitingIndex < ARRAY_SIZE(rxTaskWaiting))
				{
					TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
				}
			}
		}
#endif

		// Test whether any messages have been transmitted
		if (ir & CAN_(IR_TC))
		{
			// Check which transmit buffers have finished transmitting
			uint32_t transmitDone;
			while ((transmitDone = (~hw->REG(TXBRP)) & hw->REG(TXBTIE)) != 0)
			{
				const unsigned int bufferNumber = LowestSetBit(transmitDone);
				hw->REG(TXBTIE) &= ~((uint32_t)1 << bufferNumber);
#if !STM32H5
				if (bufferNumber < READBITS(hw, TXBC, NDTB))
				{
					// Completed transmission from a dedicated transmit buffer
					const unsigned int waitingIndex = bufferNumber + (unsigned int)TxBufferNumber::buffer0;
					if (waitingIndex < ARRAY_SIZE(txTaskWaiting))
					{
						TaskBase::GiveFromISR(txTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
					}
				}
				else
#endif
				{
					// Completed transmission from a transmit FIFO buffer
					TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo], NotifyIndices::CanDevice);
				}
			}
		}

		// Test for messages lost due to receive fifo full
		if (ir & (CAN_(IR_RF0L) | CAN_(IR_RF1L)))
		{
			++stats.messagesLost;
		}

		// Test for CAN protocol errors
		if (ir & (CAN_(IR_PED)  | CAN_(IR_PEA)))
		{
			++stats.protocolErrors;
		}

		// Test for bus off condition
		if (ir & CAN_(IR_BO))
		{
			++stats.busOffCount;
			DoHardwareInit();
			Enable();
			return;
		}

		// Test whether there are any events in the transmit event fifo
		if (ir & CAN_(IR_TEFN))
		{
			uint32_t txefs;
			while (((txefs = hw->REG(TXEFS)) & CAN_(TXEFS_EFFL_Msk)) != 0)
			{
				const uint32_t index = (txefs & CAN_(TXEFS_EFGI_Msk)) >> CAN_(TXEFS_EFGI_Pos);
				const volatile TxEvent *const elemPtr = GetTxEvent(index);
				Cache::InvalidateAfterDMAReceive(elemPtr, sizeof(TxEvent));		// to avoid reading old events
				TxEvent event;
				event.R0.val = elemPtr->R0.val;
				event.R1.val = elemPtr->R1.val;
				hw->REG(TXEFA) = index;											// doing this early instead of at the end of the loop helps
				if (event.R1.bit.ET == 1)
				{
					CanId id;
					id.SetReceivedId(event.R0.bit.ID);
					txCallback(event.R1.bit.MM, id, event.R1.bit.TXTS);
				}
				__DSB();														// probably not needed, but just in case
			}
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

#endif
