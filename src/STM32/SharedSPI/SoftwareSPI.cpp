//Author: sdavi

//SoftwareSPI

#if USE_SWSPI
#include "SoftwareSPI.h"

// On some processors we need to ensure that memory mapped I/O operations are synced to the hardware
# if STM32H7
# define SYNC_GPIO() __DSB()
#else
# define SYNC_GPIO() 
# endif

// The following three constatnts provide a rough estimate of how many clock cycles are required for one
// bit transfer. These can then be scaled based on mcu speed to adjust the actual SPI timing to match the
// requested transfer rate.

static constexpr uint32_t fastCycleCount = 28;
static constexpr uint32_t slowCycleCount = 50;
static constexpr uint32_t timingCycleCount = 18;

//#define SWSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

SoftwareSPI SoftwareSPI::SWSSP0;
SoftwareSPI SoftwareSPI::SWSSP1;
SoftwareSPI SoftwareSPI::SWSSP2;

bool SoftwareSPI::waitForTxEmpty() noexcept
{
    return false;
}


void SoftwareSPI::initPins(Pin clk, Pin miso, Pin mosi, NvicPriority priority) noexcept
{
    this->sck = clk;
    this->miso = miso;
    this->mosi = mosi;
}

//setup the master device.
void SoftwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    if(needInit || mode != clockMode || bitRate != rate)
    {
        if (sck == NoPin)
            debugPrintf("Warning: Software SPI %d clock pin is not configured, check configuration\n", (this == &SWSSP0 ? 0 : this == &SWSSP1 ? 1 : 2));
        mode = clockMode;
        // Work out what delays we need to meet the requested bit rate.
        uint32_t targetCycleNanos = 1000000/(bitRate/1000);
        uint32_t fastCycleNanos = (fastCycleCount*1000000)/(SystemCoreClock/1000);
        uint32_t slowCycleNanos = (slowCycleCount*1000000)/(SystemCoreClock/1000);
        uint32_t timingCycleNanos = (timingCycleCount*1000000)/(SystemCoreClock/1000);
        if (targetCycleNanos <= fastCycleNanos)
            delay = 0;
        else if (targetCycleNanos <= slowCycleNanos)
            delay = 1;
        else
            delay = NanosecondsToCycles(targetCycleNanos - timingCycleNanos)/2;
        //debugPrintf("SSPI mode %x bitrate %d target %d fast %d slow %d timing %d delay %d\n", clockMode, bitRate, targetCycleNanos, fastCycleNanos, slowCycleNanos, timingCycleNanos, delay );
        SetPinMode(miso, INPUT_PULLUP, false);
        SetPinMode(mosi, OUTPUT_HIGH);
        SetPinMode(sck, (mode & 2 ? OUTPUT_HIGH : OUTPUT_LOW));
        if (miso != NoPin) pin_speed(miso, GPIO_SPEED_FREQ_LOW);
        if (mosi != NoPin) pin_speed(mosi, GPIO_SPEED_FREQ_LOW);
        if (sck != NoPin) pin_speed(sck, GPIO_SPEED_FREQ_LOW);
        SYNC_GPIO();
        rate = bitRate;
        needInit = false;
    }
}


SoftwareSPI::SoftwareSPI() noexcept
    :needInit(true),sck(NoPin),mosi(NoPin),miso(NoPin),mode(0),rate(0)
{

}

spi_status_t SoftwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
{
    if (sck == NoPin) return SPI_ERROR;
    if (cs != NoPin) fastDigitalWriteLow(cs);
    uint32_t startTime = millis();
    for (uint32_t i = 0; i < len; ++i)
    {
        uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
        uint8_t dIn = (mode & 2 ? mode23TransferByte(dOut) : mode01TransferByte(dOut));
        if(rx_data != nullptr) *rx_data++ = dIn;
    }
    if (cs != NoPin) fastDigitalWriteHigh(cs);
    if (millis() - startTime > 1000) debugPrintf("Software SPI request took %dms\n", (int)(millis() - startTime));
	return SPI_OK;
}

/*
 * Simultaneously transmit and receive a byte on the SPI.
 *
 * Supports mode 0 and mode 1.
 *
 * Returns the received byte.
 
 //WikiPedia: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Example_of_bit-banging_the_master_protocol
 
 */
uint8_t SoftwareSPI::mode01TransferByte(uint8_t byte_out) noexcept
{
    uint8_t byte_in = 0;
    uint8_t bit;
    uint32_t start = GetCurrentCycles();
    if (mode & 1)
    {
        if (delay == 0)
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
           }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
            }
        }
    }
    else
    {
        if (delay == 0)
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
            }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
            }
        }
    }
    return byte_in;
}


/*
 * Simultaneously transmit and receive a byte on the SPI.
 *
 * Supports mode 2 and mode 3.
 *
 * Returns the received byte.
 
 //WikiPedia: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Example_of_bit-banging_the_master_protocol
 
 */
uint8_t SoftwareSPI::mode23TransferByte(uint8_t byte_out) noexcept
{
    uint8_t byte_in = 0;
    uint8_t bit;
    uint32_t start = GetCurrentCycles();
    if (mode & 1)
    {
        if (delay == 0)
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
           }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    //start = DelayCycles(start, 10);
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
            }

        }
    }
    else
    {
        if (delay == 0)
        {       
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
            }
        }
        else
        {
            for (bit = 0x80; bit; bit >>= 1) {
                /* Shift-out a bit to the MOSI line */
                if (mosi != NoPin)
                {
                    if (byte_out & bit)
                        fastDigitalWriteHigh(mosi);
                    else
                        fastDigitalWriteLow(mosi);
                }
                SYNC_GPIO();
                start = DelayCycles(start, delay);
                /* Pull the clock line low */
                fastDigitalWriteLow(sck);
                SYNC_GPIO();
                /* Shift-in a bit from the MISO line */
                if (miso != NoPin && fastDigitalRead(miso))
                    byte_in |= bit;
                start = DelayCycles(start, delay);
                /* Pull the clock line high */
                fastDigitalWriteHigh(sck);
                SYNC_GPIO();
            }
        }
    }
    return byte_in;
}
#endif