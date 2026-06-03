#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "CoreImp.h"
#include "SPI.h"

#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#endif
#include "spi_com.h"
extern "C" void DMA1_Stream3_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream4_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream0_IRQHandler(void) noexcept;
extern "C" void DMA1_Stream5_IRQHandler(void) noexcept;
extern "C" void SPI1_IRQHandler() noexcept;
extern "C" void SPI2_IRQHandler() noexcept;
extern "C" void SPI3_IRQHandler() noexcept;
#if STM32H7
extern "C" void DMA1_Stream1_IRQHandler() noexcept;
extern "C" void DMA1_Stream2_IRQHandler() noexcept;
extern "C" void DMA1_Stream6_IRQHandler() noexcept;
extern "C" void DMA1_Stream7_IRQHandler() noexcept;
extern "C" void SPI4_IRQHandler() noexcept;
extern "C" void SPI5_IRQHandler() noexcept;
extern "C" void SPI6_IRQHandler() noexcept;
#endif

enum class SpiIoType : uint8_t
{
	polled = 0,
    interrupt,
    dma
};

class HardwareSPI;
typedef void (*SPICallbackFunction)(HardwareSPI *spiDevice) noexcept;
class HardwareSPI: public SPI
{
public:
    HardwareSPI(SPI_TypeDef *spi) noexcept;
    HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo) noexcept;
    HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo, DMA_Stream_TypeDef* rxStream, uint32_t rxChan, IRQn_Type rxIrqNo,
                            DMA_Stream_TypeDef* txStream, uint32_t txChan, IRQn_Type txIrqNo) noexcept;
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs = NoPin) noexcept;
    bool waitForTxEmpty() noexcept;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept; // Master mode
    void configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, Pin cs = NoPin) noexcept;
    void initPins(Pin sck, Pin miso, Pin mosi, NvicPriority priority = -1) noexcept;
    void disable() noexcept;
    void flushRx() noexcept;
    void startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete, size_t minDMALen = 0) noexcept;
    void stopTransfer() noexcept;
    static HardwareSPI SSP1;
    static HardwareSPI SSP2;
    static HardwareSPI SSP3;
#if STM32H7
    static HardwareSPI SSP4;
    static HardwareSPI SSP5;
    static HardwareSPI SSP6;
#endif

    spi_t spi;
private:
    SPI_TypeDef *dev;
    IRQn_Type spiIrq;
    DMA_HandleTypeDef dmaRx;
    DMA_HandleTypeDef dmaTx;
    IRQn_Type rxIrq;
    IRQn_Type txIrq;
    Pin csPin;
    uint32_t curBitRate;
    uint32_t curBits;
    uint32_t curClockMode;
    SPICallbackFunction callback;
#ifdef RTOS
    TaskBase *waitingTask;
#endif
    bool initComplete;
    bool transferActive;
    SpiIoType ioType;
    void configureDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept;
    void initDma(NvicPriority priority) noexcept;
    static const size_t minDMAThreshold = 6;

    friend void DMA1_Stream3_IRQHandler() noexcept;
    friend void DMA1_Stream4_IRQHandler() noexcept;
    friend void DMA1_Stream0_IRQHandler() noexcept;
    friend void DMA1_Stream5_IRQHandler() noexcept;
    friend void transferComplete(HardwareSPI *spiDevice) noexcept;
    friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
    friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
    friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
    friend void SPI1_IRQHandler() noexcept __attribute__((optimize("O2")));
    friend void SPI2_IRQHandler() noexcept;
    friend void SPI3_IRQHandler() noexcept;
#if STM32H7
    friend void DMA1_Stream1_IRQHandler() noexcept;
    friend void DMA1_Stream2_IRQHandler() noexcept;
    friend void DMA1_Stream6_IRQHandler() noexcept;
    friend void DMA1_Stream7_IRQHandler() noexcept;
    friend void SPI4_IRQHandler() noexcept;
    friend void SPI5_IRQHandler() noexcept;
    friend void SPI6_IRQHandler() noexcept;
#endif
    
};

#endif
