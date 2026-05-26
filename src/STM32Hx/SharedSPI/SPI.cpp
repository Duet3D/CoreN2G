//Implement the SharedSpi as in RRF

#include "Core.h"
#include "SPI.h"

#include "SoftwareSPI.h"
#include "HardwareSPI.h"

SPI *SPI::getSSPDevice(SSPChannel channel)
{
    switch(channel)
    {
#if USE_SSP1
        case SSP1: return &HardwareSPI::SSP1; break;
#endif
#if USE_SSP2
        case SSP2: return &HardwareSPI::SSP2; break;
#endif
#if USE_SSP3
        case SSP3: return &HardwareSPI::SSP3; break;
#endif
#ifdef USE_SWSPI
        case SWSPI0: return &SoftwareSPI::SWSSP0; break;
        case SWSPI1: return &SoftwareSPI::SWSSP1; break;
        case SWSPI2: return &SoftwareSPI::SWSSP2; break;
#endif
#if USE_SSP4
        case SSP4: return &HardwareSPI::SSP4; break;
#endif
#if USE_SSP5
        case SSP5: return &HardwareSPI::SSP5; break;
#endif
#if USE_SSP6
        case SSP6: return &HardwareSPI::SSP6; break;
#endif
        default: return nullptr;
    }
}

