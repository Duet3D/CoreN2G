/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#include "Serial.h"

#include <peripheral_clk_config.h>
#include <hal_gpio.h>
#include <RTOSIface/RTOSIface.h>

#if SAME5x
# include <hri_sercom_e54.h>
#elif SAMC21
# include <hri_sercom_c21.h>
#else
# error Unsupported processor
#endif

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we default to

// Enable the clocks for the SERCOM.
void Serial::EnableSercomClock(uint8_t sercomNumber) noexcept
{
	struct SercomClockParams
	{
		uint8_t gclkFastIndex;
		uint8_t gclkSlowIndex;
		volatile uint32_t& mclkMaskReg;
		uint32_t mcklBitVal;
	};

	static const SercomClockParams SercomClockTable[] =
	{
#if SAME5x
		{ SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW, MCLK->APBAMASK.reg, MCLK_APBAMASK_SERCOM0 },
		{ SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW, MCLK->APBAMASK.reg, MCLK_APBAMASK_SERCOM1 },
		{ SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW, MCLK->APBBMASK.reg, MCLK_APBBMASK_SERCOM2 },
		{ SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW, MCLK->APBBMASK.reg, MCLK_APBBMASK_SERCOM3 },
		{ SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW, MCLK->APBDMASK.reg, MCLK_APBDMASK_SERCOM4 },
		{ SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW, MCLK->APBDMASK.reg, MCLK_APBDMASK_SERCOM5 },
		{ SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW, MCLK->APBDMASK.reg, MCLK_APBDMASK_SERCOM6 },
		{ SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW, MCLK->APBDMASK.reg, MCLK_APBDMASK_SERCOM7 },
#elif SAMC21
		{ SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM0 },
		{ SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM1 },
		{ SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM2 },
		{ SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM3 },
		{ SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM4 },
		{ SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW, MCLK->APBCMASK.reg, MCLK_APBCMASK_SERCOM5 },
#else
# error Unsupported processor
#endif
	};

	if (sercomNumber < ARRAY_SIZE(SercomClockTable))
	{
		const SercomClockParams p = SercomClockTable[sercomNumber];
		GCLK->PCHCTRL[p.gclkFastIndex].reg = GCLK_PCHCTRL_GEN(SercomFastGclkNum) | GCLK_PCHCTRL_CHEN;
		GCLK->PCHCTRL[p.gclkSlowIndex].reg = GCLK_PCHCTRL_GEN(SercomSlowGclkNum) | GCLK_PCHCTRL_CHEN;
		p.mclkMaskReg |= p.mcklBitVal;
	}
}

// Initialise the serial port. This does not set up the I/O pins. It assumes that we always transmit on pad 0.
void Serial::InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad, uint8_t txPad, UartMode uartMode
#if SAME5x
						, bool use32bitMode
#endif
					 ) noexcept
{
	EnableSercomClock(sercomNumber);
	Sercom * const sercom = GetSercom(sercomNumber);

	uint32_t ctrla = (1u << SERCOM_USART_CTRLA_DORD_Pos)				// MSB first
					 | (0u << SERCOM_USART_CTRLA_CPOL_Pos)				// use rising clock edge
					 | (0u << SERCOM_USART_CTRLA_CMODE_Pos)				// async mode
					 | (0u << SERCOM_USART_CTRLA_FORM_Pos)				// usart frame, no parity
					 | (0u << SERCOM_USART_CTRLA_SAMPA_Pos)				// sample on clocks 7-8-9
					 | ((uint32_t)rxPad << SERCOM_USART_CTRLA_RXPO_Pos)	// receive data pad
					 | ((uint32_t)txPad << SERCOM_USART_CTRLA_TXPO_Pos)	// transmit data pad
					 | (0u << SERCOM_USART_CTRLA_SAMPR_Pos)				// 16x over sampling, normal baud rate generation
#if SAME5x
					 | (0u << SERCOM_USART_CTRLA_RXINV_Pos)				// don't invert receive data
					 | (0u << SERCOM_USART_CTRLA_TXINV_Pos)				// don't invert transmitted data
#endif
					 | (0u << SERCOM_USART_CTRLA_IBON_Pos)				// don't report buffer overflow early
					 | (0u << SERCOM_USART_CTRLA_RUNSTDBY_Pos)			// don't clock during standby
					 | (1u << SERCOM_USART_CTRLA_MODE_Pos)				// use internal clock
					 | (0u << SERCOM_USART_CTRLA_ENABLE_Pos)			// not enabled
					 | (0u << SERCOM_USART_CTRLA_SWRST_Pos);			// no reset
	uint32_t ctrlb = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
	switch (uartMode)
	{
	case UartMode::Mode8N1:
	default:
		break;

	case UartMode::Mode8E1:
		ctrla |= 1u << SERCOM_USART_CTRLA_FORM_Pos;
		break;

	case UartMode::Mode8O1:
		ctrlb |= SERCOM_USART_CTRLB_PMODE;
		ctrla |= 1u << SERCOM_USART_CTRLA_FORM_Pos;
		break;
	}

	if (!hri_sercomusart_is_syncing(sercom, SERCOM_USART_SYNCBUSY_SWRST))
	{
		const uint32_t mode = ctrla & SERCOM_USART_CTRLA_MODE_Msk;
		if (hri_sercomusart_get_CTRLA_reg(sercom, SERCOM_USART_CTRLA_ENABLE))
		{
			hri_sercomusart_clear_CTRLA_ENABLE_bit(sercom);
			hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_ENABLE);
		}
		hri_sercomusart_write_CTRLA_reg(sercom, SERCOM_USART_CTRLA_SWRST | mode);
	}
	hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_SWRST);

	sercom->USART.CTRLA.reg = ctrla;
	sercom->USART.CTRLB.reg = ctrlb;
#if SAME5x
	sercom->USART.CTRLC.reg = (use32bitMode) ? SERCOM_USART_CTRLC_DATA32B(3) : 0u;
#else
	sercom->USART.CTRLC.reg = 0u;
#endif
	const uint32_t baudReg = 65536u - (((uint64_t)65536 * 16 * baudRate)/SercomFastGclkFreq);
	sercom->USART.BAUD.reg = baudReg;
	hri_sercomusart_set_CTRLA_ENABLE_bit(sercom);
	hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_ENABLE);
}

// Undo the initialisation, so that when we jump into the main firmware the USART can be initialised again
void Serial::DisableSercom(uint8_t sercomNumber) noexcept
{
	Sercom * const sercom = GetSercom(sercomNumber);
	hri_sercomusart_clear_CTRLA_ENABLE_bit(sercom);
	hri_sercomusart_set_CTRLA_SWRST_bit(sercom);
}

static void DummyHandler(void*) noexcept
{
	// Maybe we should record an exception instead of just looping?
	while (1) { }
}

#if SAMC21

static Serial::IrqFunc sercomIrq[6];
static void *sercomParam[6];

void Serial::SetSercomVector(uint8_t sercomNumber, Serial::IrqFunc f, void *param) noexcept
{
	sercomParam[sercomNumber] = param;
	sercomIrq[sercomNumber] = f;
}

void Serial::ReleaseSercomVector(uint8_t sercomNumber) noexcept
{
	sercomIrq[sercomNumber] = DummyHandler;
}

# define DEFINE_SERCOM_IRQ(_sercom) \
	void SERCOM ## _sercom ## _Handler() noexcept { sercomIrq[_sercom](sercomParam[_sercom]); }

#elif SAME5x

static Serial::IrqFunc sercomIrq[8][4];
static void *sercomParam[8];

void Serial::SetSercomVector(uint8_t sercomNumber, Serial::IrqFunc f0, Serial::IrqFunc f1, Serial::IrqFunc f2, Serial::IrqFunc f3, void *param) noexcept
{
	sercomParam[sercomNumber] = param;
	sercomIrq[sercomNumber][0] = (f0 == nullptr) ? DummyHandler : f0;
	sercomIrq[sercomNumber][1] = (f1 == nullptr) ? DummyHandler : f1;
	sercomIrq[sercomNumber][2] = (f2 == nullptr) ? DummyHandler : f2;
	sercomIrq[sercomNumber][3] = (f3 == nullptr) ? DummyHandler : f3;
}

void Serial::ReleaseSercomVector(uint8_t sercomNumber) noexcept
{
	sercomIrq[sercomNumber][0] = sercomIrq[sercomNumber][1] = sercomIrq[sercomNumber][2] = sercomIrq[sercomNumber][3] = DummyHandler;
}

# define DEFINE_SERCOM_IRQ(_sercom) \
	void SERCOM ## _sercom ## _0_Handler() noexcept { sercomIrq[_sercom][0](sercomParam[_sercom]); } \
	void SERCOM ## _sercom ## _1_Handler() noexcept { sercomIrq[_sercom][1](sercomParam[_sercom]); } \
	void SERCOM ## _sercom ## _2_Handler() noexcept { sercomIrq[_sercom][2](sercomParam[_sercom]); } \
	void SERCOM ## _sercom ## _3_Handler() noexcept { sercomIrq[_sercom][3](sercomParam[_sercom]); }

#endif

DEFINE_SERCOM_IRQ(0)
DEFINE_SERCOM_IRQ(1)
DEFINE_SERCOM_IRQ(2)
DEFINE_SERCOM_IRQ(3)
DEFINE_SERCOM_IRQ(4)
DEFINE_SERCOM_IRQ(5)

#if SAME5x

DEFINE_SERCOM_IRQ(6)
DEFINE_SERCOM_IRQ(7)

#endif

void Serial::Init() noexcept
{
#if SAMC21
	for (IrqFunc& f : sercomIrq)
	{
		f = DummyHandler;
	}
#elif SAME5x
	for (size_t i = 0; i < 8; ++i)
	{
		for (size_t j = 0; j < 4; ++j)
		{
			sercomIrq[i][j] = DummyHandler;
		}
	}
#endif
}

// End
