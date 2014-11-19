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

/*
 * Authors:
 * Dominic Clifton/Hydra - Various cleanups for Cleanflight
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#ifdef USE_USART1
static uartPort_t uartPort1;
#endif

#ifdef USE_USART2
static uartPort_t uartPort2;
#endif

#ifdef USE_USART3
static uartPort_t uartPort3;
#endif

#ifdef USE_USART1
// USART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUSART1(const serialPortConfig_t *config)
{
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    uartPort_t *self = &uartPort1;
    self->port.vTable = &uartVTable;

    self->port.baudRate = config->baudRate;

    self->port.rxBuffer = rx1Buffer;
    self->port.txBuffer = tx1Buffer;
    self->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    self->port.txBufferSize = UART1_TX_BUFFER_SIZE;

    self->USARTx = USART1;

    self->txDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;
    self->rxDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;

    self->rxDMAChannel = DMA1_Channel5;
    self->txDMAChannel = DMA1_Channel4;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // USART1_TX    PA9
    // USART1_RX    PA10
    gpio.speed = Speed_2MHz;
    if(config->mode & MODE_SINGLEWIRE) {
        gpio.pin = Pin_9;
        gpio.mode = Mode_AF_OD;
        gpioInit(GPIOA, &gpio);
    } else {
        if (config->mode & MODE_TX) {
            gpio.pin = Pin_9;
            gpio.mode = Mode_AF_PP;
            gpioInit(GPIOA, &gpio);
        }
        if (config->mode & MODE_RX) {
            gpio.pin = Pin_10;
            gpio.mode = Mode_IPU;
            gpioInit(GPIOA, &gpio);
        }
    }

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART1_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART1_TXDMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return self;
}


// USART1 Tx DMA Handler
void DMA1_Channel4_IRQHandler(void)
{
    uartPort_t *self = &uartPort1;
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(self->txDMAChannel, DISABLE);

    if (self->port.txBufferHead != self->port.txBufferTail)
        uartStartTxDMA(self);
    else
        self->txDMAEmpty = true;
}

// USART1 Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartIrqHandler(&uartPort1);
}

#endif

#ifdef USE_USART2
// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUSART2(const serialPortConfig_t *config)
{
    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    uartPort_t *self = &uartPort2;
    self->port.vTable = &uartVTable;

    self->port.baudRate = config->baudRate;

    self->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    self->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    self->port.rxBuffer = rx2Buffer;
    self->port.txBuffer = tx2Buffer;

    self->USARTx = USART2;

    self->txDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;
    self->rxDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // USART2_TX    PA2
    // USART2_RX    PA3
    gpio.speed = Speed_2MHz;
    if(config->mode & MODE_SINGLEWIRE) {
        gpio.pin = Pin_2;
        gpio.mode = Mode_AF_OD;
        gpioInit(GPIOA, &gpio);
    } else {
        if (config->mode & MODE_TX) {
            gpio.pin = Pin_2;
            gpio.mode = Mode_AF_PP;
            gpioInit(GPIOA, &gpio);
        }
        if (config->mode & MODE_RX) {
            gpio.pin = Pin_3;
            gpio.mode = Mode_IPU;
            gpioInit(GPIOA, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART2);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART2);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return self;
}


// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartIrqHandler(&uartPort2);
}

#endif

#ifdef USE_USART3
// USART3
uartPort_t *serialUSART3(const serialPortConfig_t *config)
{
    uartPort_t *self;
    static volatile uint8_t rx3Buffer[UART3_RX_BUFFER_SIZE];
    static volatile uint8_t tx3Buffer[UART3_TX_BUFFER_SIZE];
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    self = &uartPort3;
    self->port.vTable = &uartVTable;

    self->port.baudRate = config->baudRate;

    self->port.rxBuffer = rx3Buffer;
    self->port.txBuffer = tx3Buffer;
    self->port.rxBufferSize = UART3_RX_BUFFER_SIZE;
    self->port.txBufferSize = UART3_TX_BUFFER_SIZE;

    self->USARTx = USART3;

    self->txDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;
    self->rxDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;

#ifdef USART3_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(USART3_APB1_PERIPHERALS, ENABLE);
#endif
#ifdef USART3_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(USART3_APB2_PERIPHERALS, ENABLE);
#endif

    gpio.speed = Speed_2MHz;
    if(config->mode & MODE_SINGLEWIRE) {
        gpio.pin = USART3_TX_PIN;
        gpio.mode = Mode_AF_OD;
        gpioInit(GPIOA, &gpio);
    } else {
        if (config->mode & MODE_TX) {
            gpio.pin = USART3_TX_PIN;
            gpio.mode = Mode_AF_PP;
            gpioInit(GPIOA, &gpio);
        }
        if (config->mode & MODE_RX) {
            gpio.pin = USART3_RX_PIN;
            gpio.mode = Mode_IPU;
            gpioInit(GPIOA, &gpio);
        }
    }

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SERIALUART3);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SERIALUART3);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return self;
}

// USART2 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartIrqHandler(&uartPort3);
}
#endif
