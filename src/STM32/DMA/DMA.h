#ifndef DMA_H
#define DMA_H

#include "CoreImp.h"

HAL_StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength, 
                                    uint32_t Mode, uint32_t Direction, uint32_t MemInc) noexcept;
#endif
