/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

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

/*
 * DMA handlers for DMA resources that are shared between different features depending on run-time configuration.
 */
static dmaHandlers_t dmaHandlers;

void dmaNoOpHandler(DMA_Channel_TypeDef *channel)
{
    UNUSED(channel);
}

void DMA1_Channel2_IRQHandler(void)
{
    dmaHandlers.dma1Channel2IRQHandler(DMA1_Channel2);
}

void DMA1_Channel3_IRQHandler(void)
{
    dmaHandlers.dma1Channel3IRQHandler(DMA1_Channel3);
}

void DMA1_Channel6_IRQHandler(void)
{
    dmaHandlers.dma1Channel6IRQHandler(DMA1_Channel6);
}

void DMA1_Channel7_IRQHandler(void)
{
    dmaHandlers.dma1Channel7IRQHandler(DMA1_Channel7);
}

void dmaInit(void)
{
    memset(&dmaHandlers, 0, sizeof(dmaHandlers));
    dmaHandlers.dma1Channel2IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma1Channel3IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma1Channel6IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma1Channel7IRQHandler = dmaNoOpHandler;
}

void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback)
{
    switch (identifier) {
        case DMA1_CH2_HANDLER:
            dmaHandlers.dma1Channel2IRQHandler = callback;
            break;
        case DMA1_CH3_HANDLER:
            dmaHandlers.dma1Channel3IRQHandler = callback;
            break;
        case DMA1_CH6_HANDLER:
            dmaHandlers.dma1Channel6IRQHandler = callback;
            break;
        case DMA1_CH7_HANDLER:
            dmaHandlers.dma1Channel7IRQHandler = callback;
            break;
    }
}
