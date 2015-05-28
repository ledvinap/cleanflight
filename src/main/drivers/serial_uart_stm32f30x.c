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
 * Dominic Clifton - Port baseflight STM32F10x to STM32F30x for cleanflight
 * J. Ihlein - Code from FocusFlight32
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "build_config.h"

#include "platform.h"

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "drivers/io.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

// Using RX DMA disables the use of receive callbacks
#define USE_USART1_RX_DMA
//#define USE_USART2_RX_DMA
//#define USE_USART2_TX_DMA
//#define USE_USART3_RX_DMA
//#define USE_USART3_TX_DMA

#ifndef UART1_TX_IO
#define UART1_TX_IO        &IO_PA9  // PA9
#define UART1_RX_IO        &IO_PA10 // PA10
#define UART1_GPIO_AF      GPIO_AF_7
#endif

#ifndef UART2_TX_IO
#define UART2_TX_IO        &IO_PD5 // PD5
#define UART2_RX_IO        &IO_PD6 // PD6
#define UART2_GPIO_AF      GPIO_AF_7
#endif

#ifndef UART3_TX_IO
#define UART3_TX_IO        &IO_PB10 // PB10 (AF7)
#define UART3_RX_IO        &IO_PB11 // PB11 (AF7)
#define UART3_GPIO_AF      GPIO_AF_7
#endif

#ifdef USE_USART1
static uartPort_t uartPort1;
static uint8_t uartPort1RxBuffer[UART1_RX_BUFFER_SIZE];
static uint8_t uartPort1TxBuffer[UART1_TX_BUFFER_SIZE];
static const uartHwDef_t uartPort1Def = {
    .uartPort = &uartPort1,
    .rxBuffer = uartPort1RxBuffer,
    .rxBufferSize = sizeof(uartPort1RxBuffer),
    .txBuffer = uartPort1TxBuffer,
    .txBufferSize = sizeof(uartPort1TxBuffer),
    .USARTx = USART1,
    .IRQn = USART1_IRQn,
    .IRQPrio = NVIC_PRIO_SERIALUART1,
    .rxDMAChannelId = DMAId1c5,
    .IRQPrio_rxDMA = NVIC_PRIO_SERIALUART1_RXDMA,
    .txDMAChannelId = DMAId1c4,
    .IRQPrio_txDMA = NVIC_PRIO_SERIALUART1_TXDMA,
    .rcc = RCC_APB2(USART1),
    .rxCh = UART1_RX_IO,
    .txCh = UART1_TX_IO,
#ifdef UART1_RX_IO_REMAP
    .rxChRemap = UART1_RX_IO_REMAP,
#endif
#ifdef UART1_TX_IO_REMAP
    .txChRemap = UART1_TX_IO_REMAP,
#endif
    .afConfig = UART1_GPIO_AF,
};
#endif

#ifdef USE_USART2
static uartPort_t uartPort2;
static uint8_t uartPort2RxBuffer[UART1_RX_BUFFER_SIZE];
static uint8_t uartPort2TxBuffer[UART1_TX_BUFFER_SIZE];
static const uartHwDef_t uartPort2Def = {
    .uartPort = &uartPort2,
    .rxBuffer = uartPort2RxBuffer,
    .rxBufferSize = sizeof(uartPort2RxBuffer),
    .txBuffer = uartPort2TxBuffer,
    .txBufferSize = sizeof(uartPort2TxBuffer),
    .USARTx = USART2,
    .IRQn = USART2_IRQn,
    .IRQPrio = NVIC_PRIO_SERIALUART2,
    .rxDMAChannelId = DMAId1c6,
    .IRQPrio_rxDMA = NVIC_PRIO_SERIALUART1_RXDMA,
    .txDMAChannelId = DMAId1c7,
    .IRQPrio_txDMA = NVIC_PRIO_SERIALUART1_TXDMA,
    .rcc = RCC_APB1(USART2),
    .rxCh = UART2_RX_IO,
    .txCh = UART2_TX_IO,
#ifdef UART2_RX_IO_REMAP
    .rxChRemap = UART2_RX_IO_REMAP,
#endif
#ifdef UART2_TX_IO_REMAP
    .txChRemap = UART2_TX_IO_REMAP,
#endif
    .afConfig = UART2_GPIO_AF,
};
#endif

#ifdef USE_USART3
static uartPort_t uartPort3;
static uint8_t uartPort3RxBuffer[UART1_RX_BUFFER_SIZE];
static uint8_t uartPort3TxBuffer[UART1_TX_BUFFER_SIZE];
static const uartHwDef_t uartPort3Def = {
    .uartPort = &uartPort3,
    .rxBuffer = uartPort3RxBuffer,
    .rxBufferSize = sizeof(uartPort3RxBuffer),
    .txBuffer = uartPort3TxBuffer,
    .txBufferSize = sizeof(uartPort3TxBuffer),
    .USARTx = USART3,
    .IRQn = USART3_IRQn,
    .IRQPrio = NVIC_PRIO_SERIALUART3,
    .rxDMAChannelId = DMAId1c3,
    .IRQPrio_rxDMA = NVIC_PRIO_SERIALUART1_RXDMA,
    .txDMAChannelId = DMAId1c2,
    .IRQPrio_txDMA = NVIC_PRIO_SERIALUART1_TXDMA,
    .rcc = RCC_APB1(USART3),
    .rxCh = UART3_RX_IO,
    .txCh = UART3_TX_IO,
#ifdef UART3_RX_IO_REMAP
    .rxChRemap = UART3_RX_IO_REMAP,
#endif
#ifdef UART3_TX_IO_REMAP
    .txChRemap = UART3_TX_IO_REMAP,
#endif
    .afConfig = UART3_GPIO_AF,
};
#endif



void serialUSARTHwInit(uartPort_t *self, const serialPortMode_t *config)
{
    UNUSED(config);
    const uartHwDef_t *def = self->hwDef;


    self->txDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->TDR;
    self->rxDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->RDR;


    RCC_ClockCmd(def->rcc, ENABLE);
}

void usartHwConfigurePins(uartPort_t *self, const serialPortMode_t *config) {
    const ioDef_t *tx, *rx, *rxi, *txi;
    uint8_t af;
    if(config->mode & MODE_U_REMAP) {
        rx = self->hwDef->rxChRemap;
        tx = self->hwDef->txChRemap;
        rxi = self->hwDef->rxCh;
        txi = self->hwDef->txCh;
        af = self->hwDef->afConfigRemap;
        self->port.mode |= MODE_U_REMAP;
    } else {
        rx = self->hwDef->rxCh;
        tx = self->hwDef->txCh;
        rxi = self->hwDef->rxChRemap;
        txi = self->hwDef->txChRemap;
        af = self->hwDef->afConfig;
        self->port.mode &= ~MODE_U_REMAP;
    }

    if(self->port.mode & MODE_SINGLEWIRE) {
        IOConfigGPIOAF(tx, ((self->port.mode & MODE_INVERTED)
                            ? IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,  GPIO_OType_PP, GPIO_PuPd_DOWN)
                            : IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,  GPIO_OType_OD, GPIO_PuPd_UP)),
                       af);
        if(!(self->port.mode & MODE_INVERTED))
            IODigitalWrite(tx, true);   // OpenDrain output should be inactive
        // TODO - maybe allow remap
    } else {
        if ((self->port.mode & MODE_TX) && tx)
            IOConfigGPIOAF(tx, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), af);
        if ((self->port.mode & MODE_RX) && rx)
            IOConfigGPIOAF(rx, ((config->mode & MODE_INVERTED)
                                ? IO_CONFIG(GPIO_Mode_AF, 0, 0, GPIO_PuPd_DOWN)
                                : IO_CONFIG(GPIO_Mode_AF, 0, 0, GPIO_PuPd_UP)),
                           af);
        // TODO - remaped IO may have different inversion
        if (txi)
            IOConfigGPIO(rxi, IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP));
        if (rxi)
            IOConfigGPIO(txi, IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP));
    }
}

const uartHwDef_t* serialUSARTFindDef(USART_TypeDef *USARTx) {
    if(0) ;
#ifdef USE_USART1
    else if (USARTx == USART1) return &uartPort1Def;
#endif
#ifdef USE_USART2
    else if (USARTx == USART2) return &uartPort2Def;
#endif
#ifdef USE_USART3
    else if (USARTx == USART3) return &uartPort3Def;
#endif
    else return NULL;
}


#ifdef USE_USART1
// USART1 Tx DMA Handler
void DMA1_Channel4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    uartTxDMAHandler(&uartPort1);
}

void USART1_IRQHandler(void)
{
    uartIrqHandler(&uartPort1);
}

#endif

#ifdef USE_USART2
// USART2 Tx DMA Handler
void DMA1_Channel7_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC7);
    DMA_Cmd(DMA1_Channel7, DISABLE);
    uartTxDMAHandler(&uartPort2);
}

void USART2_IRQHandler(void)
{
    uartIrqHandler(&uartPort2);
}

#endif

// USART3 Tx DMA Handler
#ifdef USE_USART3
void DMA1_Channel2_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC2);
    DMA_Cmd(DMA1_Channel2, DISABLE);
    uartTxDMAHandler(&uartPort3);
}

void USART3_IRQHandler(void)
{
    uartIrqHandler(&uartPort3);
}

#endif
