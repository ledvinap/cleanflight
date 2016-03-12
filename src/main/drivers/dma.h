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

#pragma once

// (TODO) keep track of used DMA channels here
// It should be possible to dynamically allocate DMA channels according to configuration
// theese constants are not used now in souce code ..

#include <stdint.h>

#include "platform.h"

#if defined(STM32F10x)

#define DMA_ADC DMA1_Channel1
#define DMA_USART1_TX DMA1_Channel4
#define DMA_USART1_RX DMA1_Channel5
#define DMA_WS2811 DMA1_Channel6

#elif defined(STM32F30x)

#define DMA_ADC DMA1_Channel1
#define DMA_USART1_TX DMA1_Channel4
#define DMA_USART1_RX DMA1_Channel5
#define DMA_USART2_TX DMA1_Channel7
#define DMA_USART2_RX DMA1_Channel6
#define DMA_WS2811 DMA1_Channel3

#endif

typedef struct {
    DMA_Channel_TypeDef *channel;
    uint8_t irq;
} DMAInfo_t;

typedef enum {
    DMAIdNONE = 0,
    DMAId1c1, DMAId1c2, DMAId1c3, DMAId1c4, DMAId1c5, DMAId1c6, DMAId1c7,
#if defined(DMA2_Channel1)
    DMAId2c1, DMAId2c2, DMAId2c3, DMAId2c4, DMAId2c5
#endif
} DMAChannelID;

#define DMA_CHANNELS_COUNT DMAId2c5

extern DMAInfo_t DMAInfo[DMA_CHANNELS_COUNT];

void DMAInit(void);
void DMAInitNVIC(DMAChannelID id, uint8_t priority);
DMA_Channel_TypeDef* DMAGetChannel(DMAChannelID id);

typedef void (*dmaCallbackHandlerFuncPtr)(DMA_Channel_TypeDef *channel);

typedef enum {
    DMA1_CH2_HANDLER = 0,
    DMA1_CH3_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
} dmaHandlerIdentifier_e;

typedef struct dmaHandlers_s {
    dmaCallbackHandlerFuncPtr dma1Channel2IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel3IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel6IRQHandler;
    dmaCallbackHandlerFuncPtr dma1Channel7IRQHandler;
} dmaHandlers_t;

void dmaInit(void);
void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback);
