//Author: sdavi

#ifndef _ASYNCSERIAL_H_
#define _ASYNCSERIAL_H_


#include "Core.h"
#include "Stream.h"
#include "Print.h"

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif
#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 128
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
#define SERIAL_RX_BUFFER_SIZE 128
#endif


#ifdef UART_WORDLENGTH_7B
#define SERIAL_7N1 0x04
#define SERIAL_7N2 0x0C
#define SERIAL_6E1 0x22
#define SERIAL_6E2 0x2A
#define SERIAL_6O1 0x32
#define SERIAL_6O2 0x3A
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
	struct Errors
	{
		uint32_t uartOverrun,
				 framing,
				 bufferOverrun;

		Errors() noexcept : uartOverrun(0), framing(0), bufferOverrun(0)  {  }
	};



    AsyncSerial() noexcept;

    bool Configure(Pin rx, Pin tx) noexcept;
    
    void begin(uint32_t baud, uint8_t config) noexcept;
    void begin(uint32_t baud) noexcept;
    void end() noexcept;

    size_t write(const uint8_t *buffer, size_t size) noexcept override;
    size_t write(uint8_t c) noexcept override;

    int available(void) noexcept;
    
    int peek(void) noexcept;
    int read(void) noexcept;
    void flush(void) noexcept;
    using Print::write;

    int availableForWrite(void) noexcept;
    size_t canWrite() noexcept;

	void ClearTransmitBuffer() noexcept;
	void ClearReceiveBuffer() noexcept;
	void DisableTransmit() noexcept;
	void EnableTransmit() noexcept;

    bool IsConnected() noexcept;

    int8_t GetUARTPortNumber() noexcept;

    void setInterruptPriority(uint32_t priority) noexcept;
    uint32_t getInterruptPriority() noexcept;

	InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f) noexcept;
	OnTransmissionEndedFn SetOnTxEndedCallback(OnTransmissionEndedFn f, CallbackParameter cp) noexcept;

	// Get and clear the errors
	Errors GetAndClearErrors() noexcept;
    void UART_IRQHandler() noexcept;

private:
    size_t writeBlock(const uint8_t *buffer, size_t size) noexcept;
    void init(uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits) noexcept;
    void deinit() noexcept;
    void start_rx() noexcept;
    void start_tx() noexcept;
    void set_interrupt_priority(uint32_t priority) noexcept;
    int8_t get_port_number() noexcept;
    uint32_t rx_available() noexcept;
    uint32_t tx_available() noexcept;
    HAL_StatusTypeDef UART_Receive_IT() noexcept;
    HAL_StatusTypeDef UART_Transmit_IT(uint32_t isrflags) noexcept;
    InterruptCallbackFn interruptCallback;
	OnTransmissionEndedFn onTransmissionEndedFn;
	CallbackParameter onTransmissionEndedCp;
    USART_TypeDef *uart;
    UART_HandleTypeDef handle;
    PinName pin_tx;
    PinName pin_rx;
    IRQn_Type irq;
    uint8_t prio;
    bool txEnabled;
    uint8_t rx_buff[SERIAL_RX_BUFFER_SIZE];
    uint8_t tx_buff[SERIAL_TX_BUFFER_SIZE];
    volatile uint32_t rx_tail;
    volatile uint32_t tx_head;
    volatile uint32_t rx_head;
    volatile uint32_t tx_tail;

#ifdef RTOS
    volatile TaskHandle txWaitingTask;
#endif

    uint32_t rx_full;
    uint32_t rx_overrun;
    uint32_t hw_error;
};

extern AsyncSerial UART_Slot0;
extern AsyncSerial UART_Slot1;
extern AsyncSerial UART_Slot2;


#endif
