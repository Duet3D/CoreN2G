//author: Andy

#include <Core.h>
#include <CoreNotifyIndices.h>
#include "AsyncSerial.h"

#if STM32H5
# include <stm32h5xx_hal_conf.h>
# include <stm32h5xx_hal_uart.h>
#elif STM32H7
# include <stm32h7xx_hal_conf.h>
# include <stm32h7xx_hal_uart.h>
#endif

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

AsyncSerial::AsyncSerial(const UartParameters& params) noexcept
	: usart(Serial::GetUsart(params.instanceNumber)),
#ifdef RTOS
	  txWaitingTask(nullptr),
#endif
	  interruptCallback(nullptr),
	  onTransmissionEndedFn(nullptr),
	  usartNumber(params.instanceNumber), rxPin(params.rxPin), txPin(params.txPin), pinFunction(params.pinFunction),
	  txEnabled(false)
{
	txBuffer.Init(params.numTxSlots);
	rxBuffer.Init(params.numRxSlots);
}

void AsyncSerial::begin(uint32_t baud, UARTModes config) noexcept
{
    uint32_t databits = 0;
    uint32_t stopbits = 0;
    uint32_t parity = 0;

    // Manage databits
    switch ((uint8_t)config & 0x07)
    {
    case 0x02:
        databits = 6;
        break;
    case 0x04:
        databits = 7;
        break;
    case 0x06:
        databits = 8;
        break;
    default:
        databits = 0;
        break;
    }

    if (((uint8_t)config & 0x30) == 0x30)
    {
        parity = UART_PARITY_ODD;
        databits++;
    }
    else if (((uint8_t)config & 0x20) == 0x20)
    {
        parity = UART_PARITY_EVEN;
        databits++;
    }
    else
    {
        parity = UART_PARITY_NONE;
    }

    if (((uint8_t)config & 0x08) == 0x08)
    {
        stopbits = UART_STOPBITS_2;
    }
    else
    {
        stopbits = UART_STOPBITS_1;
    }

    switch (databits)
    {
#ifdef UART_WORDLENGTH_7B
    case 7:
        databits = UART_WORDLENGTH_7B;
        break;
#endif
    case 8:
        databits = UART_WORDLENGTH_8B;
        break;
    case 9:
        databits = UART_WORDLENGTH_9B;
        break;
    default:
    case 0:
        return;				// can't do much here
    }

    init( (uint32_t)baud, databits, parity, stopbits);
    if (uart != nullptr)
    {
        txEnabled = true;
        start_rx();
    }
}

void AsyncSerial::begin(uint32_t baud) noexcept
{
    begin(baud, UARTModes::SERIAL_8N1);
}

void AsyncSerial::end(void) noexcept
{
    if (usart != nullptr)
    {
        // wait for transmission of outgoing data
        flush();
        deinit();
    }
}

// Non-blocking read, return 0 if no character available
int AsyncSerial::read() noexcept
{
	uint8_t c;
	return (rxBuffer.GetItem(c)) ? c : -1;
}

int AsyncSerial::available() noexcept
{
	return rxBuffer.ItemsPresent();
}

size_t AsyncSerial::canWrite() noexcept
{
	return txBuffer.SpaceLeft();
}

size_t AsyncSerial::write(const uint8_t c) noexcept
{
	if (txEnabled && txBuffer.IsEmpty() && (usart->ISR & USART_ISR_TXE_TXFNF) != 0)
	{
		usart->TDR = c;
	}
	else
	{
		for (;;)
		{
			if (txBuffer.PutItem(c))
			{
				if (txEnabled)
				{
					usart->CR1 |= USART_CR1_TXFEIE;
				}
				break;
			}
			if (!txEnabled)
			{
				return 0;
			}
#ifdef RTOS
			txWaitingTask = RTOSIface::GetCurrentTask();
#endif
			usart->CR1 |= USART_CR1_TXFEIE;
#ifdef RTOS
			TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
		}
	}
	return 1;
}

size_t AsyncSerial::write(const uint8_t *buffer, size_t buflen) noexcept
{
	size_t ret = 0;
	for (;;)
	{
		const size_t numPut = txBuffer.PutBlock(buffer, buflen);
		buflen -= numPut;
		buffer += numPut;
		ret += numPut;
		if (!txEnabled)
		{
			break;
		}

		if (buflen == 0)
		{
			usart->CR1 |= USART_CR1_TXFEIE;
			break;
		}

#ifdef RTOS
		txWaitingTask = RTOSIface::GetCurrentTask();
#endif
		usart->CR1 |= USART_CR1_TXFEIE;
#ifdef RTOS
		TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
	}
	return ret;
}

void AsyncSerial::ClearTransmitBuffer() noexcept
{
	AtomicCriticalSectionLocker lock;
	txBuffer.Clear();
}

void AsyncSerial::ClearReceiveBuffer() noexcept
{
	AtomicCriticalSectionLocker lock;
	rxBuffer.Clear();
}

void AsyncSerial::setInterruptPriority(uint32_t priority) noexcept
{
	const IRQn irqNumber = Serial::GetUsartIRQn(usartNumber);
	NVIC_SetPriority(irqNumber, priority);
}

// FIXME we should probably implement the call back for this!
AsyncSerial::InterruptCallbackFn AsyncSerial::SetInterruptCallback(InterruptCallbackFn f) noexcept
{
	const InterruptCallbackFn _ecv_null ret = interruptCallback;
	interruptCallback = f;
	return ret;
}

AsyncSerial::OnTransmissionEndedFn AsyncSerial::SetOnTxEndedCallback(OnTransmissionEndedFn f, CallbackParameter cp) noexcept
{
    AtomicCriticalSectionLocker lock;
    const OnTransmissionEndedFn ret = onTransmissionEndedFn;
    onTransmissionEndedFn = f;
    onTransmissionEndedCp = cp;
    return ret;
}

void AsyncSerial::DisableTransmit() noexcept
{
    flush();
    txEnabled = false;
    usart->CR1 &= ~USART_CR1_TXFEIE;
}

void AsyncSerial::EnableTransmit() noexcept
{
    txEnabled = true;
    usart->CR1 |= USART_CR1_TXFEIE;
}

// Get and clear the errors
AsyncSerial::Errors AsyncSerial::GetAndClearErrors() noexcept
{
	Errors errs;
	std::swap(errs, errors);
	return errs;
}

void AsyncSerial::init(uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits) noexcept
{
    UART_HandleTypeDef *huart = &(handle);
    uint8_t index = 0;
    /* Determine the U(S)ART peripheral to use (USART1, USART2, ...) */
    void *tx = pinmap_peripheral(pin_tx, PinMap_UART_TX);
    void *rx = pinmap_peripheral(pin_rx, PinMap_UART_RX);

    /* Pins Rx/Tx must not be NP */
    if (rx == NP || tx == NP) {
        debugPrintf("ERROR: at least one UART pin has no peripheral\n");
        return;
    }
    hw_error = rx_full = rx_overrun = 0;

    /*
     * Get the peripheral name (USART1, USART2, ...) from the pin
     * and assign it to the ect
     */
    uart = (USART_TypeDef *)pinmap_merge_peripheral(tx, rx);

    if (uart == NP) {
        debugPrintf("ERROR: U(S)ART pins mismatch\n");
        return;
    }

    /* Enable USART clock */
#if defined(STM32F091xC) || defined (STM32F098xx)
    /* Enable SYSCFG Clock */
    /* Required to get SYSCFG interrupt status register */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
#endif

    /* Configure UART GPIO pins */
    pinmap_pinout(pin_tx, PinMap_UART_TX);
    pinmap_pinout(pin_rx, PinMap_UART_RX);

    /* Configure uart */
    handlers[index] = this;
    handle.Instance = (USART_TypeDef *)(uart);
    handle.Init.BaudRate = baudrate;
    handle.Init.WordLength = databits;
    handle.Init.StopBits = stopbits;
    handle.Init.Parity = parity;
    handle.Init.Mode = UART_MODE_TX_RX;
    handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle.Init.OverSampling = UART_OVERSAMPLING_16;
#if (defined(STM32H7xx))
    handle.FifoMode = USART_FIFOMODE_DISABLE;
#endif
#if !defined(STM32F1xx) && !defined(STM32F2xx) && !defined(STM32F4xx)\
 && !defined(STM32L1xx)
    handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif
#ifdef UART_ONE_BIT_SAMPLE_DISABLE
    handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif


    if (HAL_UART_Init(huart) != HAL_OK) {
        return;
    }
    if (prio != 0)
    {
        set_interrupt_priority(prio);
    }
    HAL_NVIC_EnableIRQ(irq);
}

void AsyncSerial::deinit() noexcept
{
    /* Reset UART and disable clock */
    HAL_NVIC_DisableIRQ(irq);
#if defined(USART1_BASE)
    if (uart == USART1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE();
    }
#endif
#if defined(USART6_BASE)
    else if (uart == USART6) {
        __HAL_RCC_USART6_FORCE_RESET();
        __HAL_RCC_USART6_RELEASE_RESET();
        __HAL_RCC_USART6_CLK_DISABLE();
    }
#endif
    HAL_UART_DeInit(&handle);
}

void AsyncSerial::CommonInterrupt(void *param) noexcept
{
	((AsyncSerial*)param)->Interrupt();
}

void AsyncSerial::Interrupt() noexcept
{
    const uint32_t isrflags = usart->ISR;
    const uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));

    while ((isrflags & USART_ISR_RXNE) != 0)
    {
		const char c = usart->RDR;
		if (c == interruptSeq[numInterruptBytesMatched])
		{
			++numInterruptBytesMatched;
			if (numInterruptBytesMatched == ARRAY_SIZE(interruptSeq))
			{
				numInterruptBytesMatched = 0;
				if (interruptCallback != nullptr)
				{
					interruptCallback(this);
				}
			}
		}
		else
		{
			numInterruptBytesMatched = 0;
		}

		if (!rxBuffer.PutItem(c))
		{
			++errors.bufferOverrun;
		}
    }

    /* If some errors occur */
    if (errorflags != 0)
    {
        /* UART frame error interrupt occurred --------------------------------------*/
        if ((isrflags & USART_ISR_FE) != 0U)
        {
        	++errors.framing;
        }

        /* UART Over-Run interrupt occurred -----------------------------------------*/
        if ((isrflags & USART_ISR_ORE) != 0U)
        {
        	++errors.uartOverrun;
        }
        usart->ICR = errorflags;
    }

    /* UART in mode Transmitter ------------------------------------------------*/
    if (((isrflags & USART_ISR_TXE) != RESET) && txEnabled)
    {
		uint8_t c;
		if (txBuffer.GetItem(c))
		{
			usart->TDR = c;
#ifdef RTOS
			if (txWaitingTask != nullptr && txBuffer.SpaceLeft() >= txBuffer.GetCapacity()/2)
			{
				TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
				txWaitingTask = nullptr;
			}
#endif
		}
		else
		{
			usart->CR1 &= ~USART_CR1_TXFEIE;
#ifdef RTOS
			if (txWaitingTask != nullptr)
			{
				TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
				txWaitingTask = nullptr;
			}
#endif
		}
    }
}

// End
