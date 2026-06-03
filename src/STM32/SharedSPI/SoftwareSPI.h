#ifndef SOFTWARESPI_H
#define SOFTWARESPI_H

#include "CoreImp.h"
#include "variant.h"
#include "SPI.h"

class SoftwareSPI: public SPI
{
public:
    SoftwareSPI();
    spi_status_t transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs = NoPin) noexcept;
    void configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept; // Master mode
    void initPins(Pin sck, Pin miso, Pin mosi, NvicPriority priority = -1) noexcept;
    bool waitForTxEmpty() noexcept;
    
    static SoftwareSPI SWSSP0;
    static SoftwareSPI SWSSP1;
    static SoftwareSPI SWSSP2;

private:
    
    uint8_t mode01TransferByte(uint8_t byte_out) noexcept;
    uint8_t mode23TransferByte(uint8_t byte_out) noexcept;
    
    bool needInit;
    Pin sck;
    Pin mosi;
    Pin miso;
    uint32_t mode;
    uint32_t delay;
    uint32_t rate;
};




#endif
