#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/utils.h"

#include "nvic.h"

#include "dma.h"

const DMAInfo_t DMAInfoConst[DMA_CHANNELS_COUNT] = {
    [DMAIdNONE] = {NULL, 0},
#define _DEF(dma,ch) [DMAId ## dma ##  c ## ch] = {DMA ## dma ## _Channel ## ch,  DMA ## dma ## _Channel ## ch ## _IRQn}
    _DEF(1,1),
    _DEF(1,2),
    _DEF(1,3),
    _DEF(1,4),
    _DEF(1,5),
    _DEF(1,6),
    _DEF(1,7),
#if defined(STM32F10X_HD)
    _DEF(2,1),
    _DEF(2,2),
    _DEF(2,3),
    _DEF(2,4),
    _DEF(2,5),
#endif
#undef _DEF
};

void DMAInitNVIC(DMAChannelID id, uint8_t irqPriority)
{
    if(id == DMAIdNONE) return;
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMAInfoConst[id].irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

DMA_Channel_TypeDef* DMAGetChannel(DMAChannelID id)
{
    if(id >= DMA_CHANNELS_COUNT)
        return NULL;
    return DMAInfoConst[id].channel;
}

void DMAInit(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}
