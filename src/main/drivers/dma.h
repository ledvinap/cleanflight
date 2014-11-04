#pragma once

// (TODO) keep track of used DMA channels here
// It should be possible to dynamically allocate DMA channels according to configuration
// theese constants are not used now in souce code .. 

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

