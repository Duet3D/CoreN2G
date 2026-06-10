//Author: sdavi

#ifndef _ASYNCSERIAL_H_
#define _ASYNCSERIAL_H_


#include "CoreIO.h"
#include "Stream.h"
#include <General/RingBuffer.h>
#include "Serial.h"
#include <UART/UartParameters.h>

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif

#ifdef UART_WORDLENGTH_7B
# define SERIAL_7N1 0x04
# define SERIAL_7N2 0x0C
# define SERIAL_6E1 0x22
# define SERIAL_6E2 0x2A
# define SERIAL_6O1 0x32
# define SERIAL_6O2 0x3A
#endif
#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

class AsyncSerial : public Stream
{
public:
	typedef void (*InterruptCallbackFn)(AsyncSerial*) noexcept;
	typedef void (*OnTransmissionEndedFn)(CallbackParameter cp) noexcept;

	union Errors
	{
		uint32_t all;
		struct
		{
			uint32_t uartOverrun : 11,
					 framing : 11,
					 bufferOverrun : 10;
		};
		Errors() noexcept { all = 0; }
	};

    AsyncSerial(const UartParameters& params) noexcept;

	// Overridden virtual functions
	int available(void) noexcept;
	int read(void) noexcept;
	void flush(void) noexcept;
	size_t canWrite() noexcept;

	size_t write(uint8_t c) noexcept override;
	size_t write(const uint8_t *buffer, size_t size) noexcept override;

	void ClearTransmitBuffer() noexcept;
	void ClearReceiveBuffer() noexcept;
	void DisableTransmit() noexcept;
	void EnableTransmit() noexcept;

	// Compatibility functions
    void begin(uint32_t baud) noexcept;
    void end() noexcept;
	void setInterruptPriority(uint32_t priority) noexcept;

	InterruptCallbackFn _ecv_null SetInterruptCallback(InterruptCallbackFn _ecv_null f) noexcept;
	OnTransmissionEndedFn _ecv_null SetOnTxEndedCallback(OnTransmissionEndedFn _ecv_null f, CallbackParameter cp) noexcept;

	// Get and clear the errors
	Errors GetAndClearErrors() noexcept;

private:
	RingBuffer<uint8_t> txBuffer;
	RingBuffer<uint8_t> rxBuffer;
	USART_TypeDef * const usart;
#ifdef RTOS
	volatile TaskHandle txWaitingTask;
#endif
    InterruptCallbackFn interruptCallback;

	void Interrupt() noexcept;
	static void CommonInterrupt(void *param) noexcept;

	OnTransmissionEndedFn _ecv_null onTransmissionEndedFn;
	CallbackParameter _ecv_null onTransmissionEndedCp;

	Errors errors;
	const uint8_t usartNumber;
	const Pin rxPin;
	const Pin txPin;
	const GpioPinFunction pinFunction;

	uint8_t numInterruptBytesMatched;
    bool bufferOverrunPending;
    bool txEnabled;

    static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif
