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

#include <platform.h>

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"
#include "serial_uart_stm32f30x.h"


// Using RX DMA disables the use of receive callbacks
//#define USE_USART1_RX_DMA
//#define USE_USART2_RX_DMA
//#define USE_USART2_TX_DMA
//#define USE_USART3_RX_DMA
//#define USE_USART3_TX_DMA

#ifndef UART1_TX_IO
#define UART1_TX_IO        PA9
#define UART1_RX_IO        PA10
#define UART1_GPIO_AF      GPIO_AF_7
#endif

#ifndef UART2_TX_IO
#define UART2_TX_IO        PD5
#define UART2_RX_IO        PD6
#define UART2_GPIO_AF      GPIO_AF_7
#endif

#ifndef UART3_TX_IO
#define UART3_TX_IO        PB10
#define UART3_RX_IO        PB11
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
    .rxCh = DEFIO_TAG(UART1_RX_IO),
    .txCh = DEFIO_TAG(UART1_TX_IO),
#ifdef UART1_RX_REMAP_IO
    .rxChRemap = DEFIO_TAG(UART1_RX_REMAP_IO),
#endif
#ifdef UART1_TX_REMAP_IO
    .txChRemap = DEFIO_TAG(UART1_TX_REMAP_IO),
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
    .rxCh = DEFIO_TAG(UART2_RX_IO),
    .txCh = DEFIO_TAG(UART2_TX_IO),
#ifdef UART2_RX_REMAP_IO
    .rxChRemap = DEFIO_TAG(UART2_RX_REMAP_IO),
#endif
#ifdef UART2_TX_REMAP_IO
    .txChRemap = DEFIO_TAG(UART2_TX_REMAP_IO),
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
    .rxCh = DEFIO_TAG(UART3_RX_IO),
    .txCh = DEFIO_TAG(UART3_TX_IO),
#ifdef UART3_RX_REMAP_IO
    .rxChRemap = DEFIO_TAG(UART3_RX_REMAP_IO),
#endif
#ifdef UART3_TX_REMAP_IO
    .txChRemap = DEFIO_TAG(UART3_TX_REMAP_IO),
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
    IO_t tx, rx, rxi, txi;
    uint8_t af;
    // allow remap only if remap pins are defined
    if(config->mode & MODE_U_REMAP && self->hwDef->rxChRemap && self->hwDef->txChRemap) {
        rx = IOGetByTag(self->hwDef->rxChRemap);
        tx = IOGetByTag(self->hwDef->txChRemap);
        rxi = IOGetByTag(self->hwDef->rxCh);
        txi = IOGetByTag(self->hwDef->txCh);
        af = self->hwDef->afConfigRemap;
        self->port.mode |= MODE_U_REMAP;
    } else {
        rx = IOGetByTag(self->hwDef->rxCh);
        tx = IOGetByTag(self->hwDef->txCh);
        rxi = IOGetByTag(self->hwDef->rxChRemap);
        txi = IOGetByTag(self->hwDef->txChRemap);
        af = self->hwDef->afConfig;
        self->port.mode &= ~MODE_U_REMAP;
    }

    if(self->port.mode & MODE_SINGLEWIRE) {
        IOInit(tx, OWNER_SERIAL_RXTX, RESOURCE_USART);
        IOConfigGPIOAF(tx, ((self->port.mode & MODE_INVERTED)
                            ? IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,  GPIO_OType_PP, GPIO_PuPd_DOWN)
                            : IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,  GPIO_OType_OD, GPIO_PuPd_UP)),
                       af);
        if(!(self->port.mode & MODE_INVERTED))
            IOWrite(tx, true);   // OpenDrain output should be inactive
        // TODO - maybe allow remap
    } else {
        IOInit(tx, OWNER_SERIAL_TX, RESOURCE_USART);
        IOInit(rx, OWNER_SERIAL_RX, RESOURCE_USART);
        if ((self->port.mode & MODE_TX) && tx)
            IOConfigGPIOAF(tx, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), af);
        if ((self->port.mode & MODE_RX) && rx)
            IOConfigGPIOAF(rx, ((config->mode & MODE_INVERTED)
                                ? IO_CONFIG(GPIO_Mode_AF, 0, 0, GPIO_PuPd_DOWN)
                                : IO_CONFIG(GPIO_Mode_AF, 0, 0, GPIO_PuPd_UP)),
                           af);
        // TODO - remaped IO may have different inversion
        // TODO - unclaimed IO should be only weakly initialized
        // do not claim remap IO, but configure PU/PD if it is not claimed yet
        if (rxi && IOGetOwner(rxi) == OWNER_FREE)
            IOConfigGPIO(rxi, IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP));
        if (txi && IOGetOwner(txi) == OWNER_FREE)
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


#ifdef USE_UART4

// UART4 Rx/Tx IRQ Handler
void UART4_IRQHandler(void)
{
    uartPort_t *s = &uartPort4;
    usartIrqHandler(&uartPort4);
}
#endif

#ifdef USE_UART5
// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &uartPort5;
    usartIrqHandler(&uartPort5);
}
#endif
