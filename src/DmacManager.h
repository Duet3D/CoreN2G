/*
 * Dmac.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_DMACMANAGER_H_
#define SRC_HARDWARE_DMACMANAGER_H_

#include <CoreIO.h>

#if SAME5x
constexpr unsigned int NumDmaChannelsSupported = 15;	// max is 32
#elif SAMC21
constexpr unsigned int NumDmaChannelsSupported = 8;		// max is 12
#elif SAME70
constexpr unsigned int NumDmaChannelsSupported = 10;	// max for SAME70 is 24
#elif STM32
constexpr unsigned int NumDmaChannelsSupported = 16;	// each DMAC has 8 channels and there are usually 2 of them
#elif RPXXXX
constexpr unsigned int NumDmaChannelsSupported = 8;		// max is 12
#endif

// Status code indicating why a DMAC callback is happening
enum class DmaCallbackReason : uint8_t
{
	none = 0,
#if SAME5x || SAMC21
	error = DMAC_CHINTFLAG_TERR,
	complete = DMAC_CHINTFLAG_TCMPL,
	completeAndError = DMAC_CHINTFLAG_TERR | DMAC_CHINTFLAG_TCMPL,
#elif SAME70
	complete = 1,
#elif STM32
	complete = 1,
#elif RPXXXX
	complete = 1,
	error = 2,
	completeAndError = 3,
#endif
};

typedef void (*DmaCallbackFunction)(CallbackParameter cb, DmaCallbackReason reason) noexcept;

enum class DmaTrigSource : uint8_t
{
#if SAME5x

	disable = 0,
	rtc,
	dsu_dcc0, dsu_dcc1,

	sercom0_rx, sercom0_tx, sercom1_rx, sercom1_tx, sercom2_rx, sercom2_tx, sercom3_rx, sercom3_tx,
	sercom4_rx, sercom4_tx, sercom5_rx, sercom5_tx, sercom6_rx, sercom6_tx, sercom7_rx, sercom7_tx,

	can0_debug,
	can1_debug,

	tcc0_ovf = 0x16, tcc0_mc,
	tcc1_ovf = 0x1D, tcc1_mc,
	tcc2_ovf = 0x22, tcc2_mc,
	tcc3_ovf = 0x26, tcc3_mc,
	tcc4_ovf = 0x29, tcc4_mc,

	tc0_ovf = 0x2C, tc0_mc,
	tc1_ovf = 0x2F, tc1_mc,
	tc2_ovf = 0x32, tc2_mc,
	tc3_ovf = 0x35, tc3_mc,
	tc4_ovf = 0x38, tc4_mc,
	tc5_ovf = 0x3B, tc5_mc,
	tc6_ovf = 0x3E, tc6_mc,
	tc7_ovf = 0x40, tc7_mc,

	adc0_resrdy = 0x44, adc0_seq, adc1_resrdy, adc1_seq,

	dac_empty = 0x48,
	dac_resrdy = 0x4A,

	i2s_rx = 0x4C,
	i2s_tx = 0x4E,

	pcc = 0x50,

	aes_wr = 0x51, aes_rd,

	qspi_rx = 0x53, qspi_tx

#elif SAMC21

	disable = 0,
	tsens,

	sercom0_rx, sercom0_tx, sercom1_rx, sercom1_tx, sercom2_rx, sercom2_tx, sercom3_rx, sercom3_tx,
	sercom4_rx, sercom4_tx, sercom5_rx, sercom5_tx,

	can0_debug, can1_debug,

	tcc0_ovf, tcc0_mc0, tcc0_mc1, tcc0_mc2, tcc0_mc3,
	tcc1_ovf, tcc1_mc0, tcc1_mc1,
	tcc2_ovf, tcc2_mc0, tcc2_mc1,

	tc0_ovf, tc0_mc0, tc0_mc1, tc1_ovf, tc1_mc0, tc1_mc1, tc2_ovf, tc2_mc0, tc2_mc1,
	tc3_ovf, tc3_mc0, tc3_mc1, tc4_ovf, tc4_mc0, tc4_mc1,

	adc0_resrdy, adc1_resrdy, sdadc_resrdy,

	dac_empty,
	ptc_eoc, ptc_wcomp, ptc_seq,

# if 0	// these are only available on the SAMC21N, which we don't support
	sercom6_rx, sercom6_tx, sercom7_rx, sercom7_tx,
	tc5_ovf, tc5_mc0, tc5_mc1, tc6_ovf, tc6_mc0, tc6_mc1, tc7_ovf, tc7_mc0, tc7_mc1
# endif

#elif SAME70 || SAM4E || SAM4S
	hsmci = 0,	// both transmit and receive
	spi0tx, spi0rx, spi1tx, spi1rx, qspitx, qspirx,
	usart0tx, usart0rx, usart1tx, usart1rx, usart2tx, usart2rx,
	pwm0tx,
	twihs0tx, twihs0rx, twihs1tx, twihs1rx, twihs2tx, twihs2rx,
	uart0tx, uart0rx, uart1tx, uart1rx, uart2tx, uart2rx, uart3tx, uart3rx, uart4tx, uart4rx,
	dacctx,
	// ID 31 does not appear in the table
	ssctx = 32, sscrx,
	pioarx,
	afec0rx, afec1rx,
	aestx, aesrx,
	pwm1tx,
	tc0rx, tc3rx, tc6rx, tc9rx,
	i2sc0txl, i2sc0rxl, i2sc1txl, i2sc1rxl, i2sc0txr, i2sc0rxr, i2sc1txr, i2sc1rxr,
#elif RPXXXX
	pio0tx0, pio0tx1, pio0tx2, pio0tx3,
	pio0rx0, pio0rx1, pio0rx2, pio0rx3,
	pio1tx0, pio1tx1, pio1tx2, pio1tx3,
	pio1rx0, pio1rx1, pio1rx2, pio1rx3,
	spi0tx, spi0rx, spi1tx, spi1rx,
	uart0tx, uart0rx, uart1tx, uart1rx,
	pwmwrap0, pwmwrap1, pwmwrap2, pwmwrap3, pwmwrap4, pwmwrap5, pwmwrap6, pwmwrap7,
	i2c0tx, i2c0rx, i2c1tx, i2c1rx,
	adc, xipstream, xipssitx, xipssirx
#elif STM32H5
	adc1 = 0, adc2, dac1_ch1, dac1_ch2, tim6_upd, tim7_upd, spi1_rx, spi1_tx,				// 0-7
	spi2_rx, spi2_tx, spi3_rx, spi3_tx, i2c1_rx, i2c1_tx,									// 8-13
	i2c2_rx = 15, i2c2_tx,																	// 15-16
	i2c3_rx = 18, i2c3_tx,																	// 18-19
	usart1_rx = 21, usart1_tx, usart2_rx, usart2_tx, usart3_rx, usart3_tx,					// 21-26
	uart4_rx = 27, uart4_tx, uart5_rx, uart5_tx, usart6_rx, usart6_tx, 						// 27-32																	// 32-39
	lpuart1_rx = 45, lpuart1_tx, spi4_rx, spi4_tx,											// 45-48
	ospi1 = 57, tim1_cc1, tim1_cc2, tim1_cc3, tim1_cc4, tim1_upd, tim1_trg, tim1_com,		// 57-64
	tim8_cc1 = 65, tim8_cc2, tim8_cc3, tim8_cc4, tim8_upd, tim8_tig, tim8_com,				// 65-71
	tim2_cc1 = 72, tim2_cc2, tim2_cc3, tim2_cc4, tim2_upd,									// 72-76
	tim3_cc1 = 77, tim3_cc2, tim3_cc3, tim3_cc4, tim3_upd, tim3_trg,						// 77-82
	tim4_cc1 = 83, tim4_cc2, tim4_cc3, tim4_cc4, tim4_upd,									// 83-87
	tim5_cc1 = 88, tim5_cc2, tim5_cc3, tim5_cc4, tim5_upd, tim5_trg,						// 88-93
	tim15_cc1 = 94, tim15_upd, tim15_trg, tim15_com,										// 94-97
	lptim1_ic1 = 102, lptim1_ic2, lptim1_ue, lptim2_ic1, lptim2_ic2, lptim2_ue,				// 102-107
	dcmi_or_pssi = 108, aes_out, aes_in, hash_in, ucpd1_rx, ucpd1_tx,						// 108-113
	saes_out = 118, saes_in, i3c1_rx, i3c1_tx, i3c1_tc, i3c1_rs,							// 118-123
	i3c2_rx = 136, i3c2_tx, i3c2_tc, i3c2_rs,												// 136-139
#elif STM32H7
	// These are the assignments for DMAMUX1. DMAMUX2 has fewer inputs.
	adc1 = 9, adc2,
	tim1_ch1 = 11, tim1_ch2, tim1_ch3, tim1_ch4, tim1_up, tim1_trig, tim1_com,
	tim2_ch1 = 18, tim2_ch2, tim2_ch3, tim2_ch4, tim2_up,
	tim3_ch1 = 23, tim3_ch2, tim3_ch3, tim3_ch4, tim3_up, tim3_trig,
	tim4_ch1 = 29, tim4_ch2, tim4_ch3, tim4_up,
	i2c1_rx = 33, i2c1_tx, i2c2_rx, i2c2_tx,
	spi1_rx = 37, spi1_tx, spi2_rx, spi2_tx,
	usart1_rx = 41, usart1_tx, usart2_rx, usart2_tx, usart3_rx, usart3_tx,
	tim8_ch1 = 47, tim8_ch2, tim8_ch3, tim8_ch4, tim8_up, tim8_trig, tim8_com,
	// 54 is reserved
	tim5_ch1 = 55, tim5_ch2, tim5_ch3, tim5_ch4, tim5_up, tim5_trig,
	spi3_rx = 61, spi3_tx, uart4_rx, uart4_tx, uart5_rx, uart5_tx,
	dac_ch1 = 67, dac_ch2, tim6_up, tim7_up, usart6_rx, usart6_tx,
	i2c3_rx = 73, i2c3_tx, dcmi, cryp_in, cryp_out, hash_in,
	uart7_rx = 79, uart7_tx, uart8_rx, uart8_tx,
	spi4_rx = 83, spi4_tx, spi5_rx, spi5_tx,
	sai1a = 87, sai1b, sai2a, sai2b, swpmi_rx, swpmi_tx, spdifrx_dat, spdifrx_ctrl,
# if defined(STM32H743xx)
	hr_req_1 = 95, hr_req_2, hr_req_3, hr_req_4, hr_req_5, hr_req_6,
# endif
	dfsdm1_0 = 101, dfsdm1_1, dfsdm1_2, dfsdm1_3,
	tim15_ch1 = 105, tim15_up, tim15_trig, tim15_com, tim16_ch1, tim16_up, tim17_ch1, tim17_up,
# if defined(STM32H743xx)
	sai3_a = 113, sai3_b,
# endif
	adc3,
# if defined(STM32H723xx)
	uart9_rx = 116, uart9_tx, uart10_rx, uart10_tx, fmac_rd, fmac_wr,
	cordic_rd = 122, cordic_wr, i2c5_rx, i2c5_tx,
	tim23_ch1 = 126, tim23_ch2, tim23_ch3,
# endif
#else
# error Unsupported processor
#endif
};

#if SAMC21
static_assert((uint8_t)DmaTrigSource::ptc_seq == 0x30, "Error in DmaTrigSource enumeration");
#endif

#if SAME5x || SAMC21

// The following works for all sercoms on the SAME51 and sercoms 1 to 5 on the SAMC21. We don't support sercoms 6-7 on the SAMC21 because they only exist on the 100-pin version.
static inline uint8_t GetSercomTxTrigSource(uint8_t sercomNumber) noexcept
{
	return (uint8_t)DmaTrigSource::sercom0_tx + (sercomNumber * 2);
}

// The following works for all sercoms on the SAME51 and sercoms 1 to 5 on the SAMC21. We don't support sercoms 6-7 on the SAMC21 because they only exist on the 100-pin version.
static inline uint8_t GetSercomRxTrigSource(uint8_t sercomNumber) noexcept
{
	return (uint8_t)DmaTrigSource::sercom0_rx + (sercomNumber * 2);
}

#endif

#if STM32

// Get the DMA trigger source for SPI transmit
static inline DmaTrigSource GetSpiTxTrigSource(uint8_t spiInstanceNumber) noexcept
{
	constexpr DmaTrigSource SpiTxTrigSources[] =
		{ 	DmaTrigSource::spi1_tx, DmaTrigSource::spi2_tx, DmaTrigSource::spi3_tx, DmaTrigSource::spi4_tx,
# if STM32H7
			DmaTrigSource::spi5_tx,
# endif
		};
	return SpiTxTrigSources[spiInstanceNumber - 1];
}

// Get the DMA trigger source for SPI receive
static inline DmaTrigSource GetSpiRxTrigSource(uint8_t spiInstanceNumber) noexcept
{
	constexpr DmaTrigSource SpiRxTrigSources[] =
		{ 	DmaTrigSource::spi1_rx, DmaTrigSource::spi2_rx, DmaTrigSource::spi3_rx, DmaTrigSource::spi4_rx,
# if STM32H7
			DmaTrigSource::spi5_rx,
# endif
		};
	return SpiRxTrigSources[spiInstanceNumber - 1];
}

// Get the DMA trigger source for USART or UART transmit
static inline DmaTrigSource GetUartTxTrigSource(uint8_t uartInstanceNumber) noexcept
{
	constexpr DmaTrigSource UartTxTrigSources[] =
		{ 	DmaTrigSource::usart1_tx, DmaTrigSource::usart2_tx, DmaTrigSource::usart3_tx, DmaTrigSource::uart4_tx, DmaTrigSource::uart5_tx, DmaTrigSource::usart6_tx,
# if STM32H7
			DmaTrigSource::uart7_tx, DmaTrigSource::uart8_tx,
#  if defined(STM32H723xx)
			DmaTrigSource::uart9_tx, DmaTrigSource::uart10_tx,
#  endif
# endif
		};
	return UartTxTrigSources[uartInstanceNumber - 1];
}

// Get the DMA trigger source for USART or UART receive
static inline DmaTrigSource GetUartRxTrigSource(uint8_t uartInstanceNumber) noexcept
{
	constexpr DmaTrigSource UartRxTrigSources[] =
		{ 	DmaTrigSource::usart1_rx, DmaTrigSource::usart2_rx, DmaTrigSource::usart3_rx, DmaTrigSource::uart4_rx, DmaTrigSource::uart5_rx, DmaTrigSource::usart6_rx,
# if STM32H7
			DmaTrigSource::uart7_rx, DmaTrigSource::uart8_rx,
#  if defined(STM32H723xx)
			DmaTrigSource::uart9_rx, DmaTrigSource::uart10_rx,
#  endif
# endif
		};
	return UartRxTrigSources[uartInstanceNumber - 1];
}

#endif

namespace DmacManager
{
	void Init() noexcept;
	void SetBtctrl(DmaChannel channel, uint32_t val) noexcept;								// warning: call SetBtctrl, SetSourceAddress and SetDestinationAddress BEFORE SetDataLength!
	void SetSourceAddress(DmaChannel channel, const volatile void *const src) noexcept;		// warning: call SetBtctrl, SetSourceAddress and SetDestinationAddress BEFORE SetDataLength!
	void SetDestinationAddress(DmaChannel channel, volatile void *const dst) noexcept;		// warning: call SetBtctrl, SetSourceAddress and SetDestinationAddress BEFORE SetDataLength!
	void SetDataLength(DmaChannel channel, uint32_t amount) noexcept;						// warning: call SetBtctrl, SetSourceAddress and SetDestinationAddress BEFORE SetDataLength!
	void SetTriggerSource(DmaChannel channel, DmaTrigSource source) noexcept;

#if SAME5x || SAMC21
	void SetTriggerSourceSercomTx(DmaChannel channel, uint8_t sercomNumber) noexcept;
	void SetTriggerSourceSercomRx(DmaChannel channel, uint8_t sercomNumber) noexcept;
	void SetArbitrationLevel(DmaChannel channel, uint8_t level) noexcept;
	uint16_t GetBytesTransferred(DmaChannel channel) noexcept;
#endif

	void EnableChannel(DmaChannel channel, DmaPriority priority) noexcept;
	bool DisableChannel(DmaChannel channel) noexcept;
	bool SuspendChannel(DmaChannel channel) noexcept;
	void ResumeChannel(DmaChannel channel) noexcept;
	void SetInterruptCallback(DmaChannel channel, DmaCallbackFunction fn, CallbackParameter param) noexcept;
	void EnableCompletedInterrupt(DmaChannel channel) noexcept;
	void DisableCompletedInterrupt(DmaChannel channel) noexcept;
	uint32_t GetAndClearChannelStatus(DmaChannel channel) noexcept;							// the meaning of the result depends on the processor
}

#endif /* SRC_HARDWARE_DMACMANAGER_H_ */
