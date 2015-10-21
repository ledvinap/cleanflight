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
#include "build_config.h"

#include "system.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "nvic.h"
#include "dma.h"

//#include "timer.h" // TODO - should be in IO.h

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

// TODO - define defaults for 103 USART mapping

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
    .rxCh = IO_TAG(USART1_RX_IO),
    .txCh = IO_TAG(USART1_TX_IO),
#ifdef USART1_RX_REMAP_IO
    .rxChRemap = IO_TAG(USART1_RX_REMAPI_IO),
#endif
#ifdef USART1_TX_REMAP_IO
    .txChRemap = IO_TAG(USART1_TX_REMAP_IO),
#endif
    .remap = GPIO_Remap_USART1,
};
#endif

#ifdef USE_USART2
static uartPort_t uartPort2;
static uint8_t uartPort2RxBuffer[UART2_RX_BUFFER_SIZE];
static uint8_t uartPort2TxBuffer[UART2_TX_BUFFER_SIZE];
static const uartHwDef_t uartPort2Def = {
    .uartPort = &uartPort2,
    .rxBuffer = uartPort2RxBuffer,
    .rxBufferSize = sizeof(uartPort2RxBuffer),
    .txBuffer = uartPort2TxBuffer,
    .txBufferSize = sizeof(uartPort2TxBuffer),
    .USARTx = USART2,
    .IRQn = USART2_IRQn,
    .IRQPrio = NVIC_PRIO_SERIALUART2,
    .rcc = RCC_APB1(USART2),
    .rxCh = IO_TAG(USART2_RX_IO),
    .txCh = IO_TAG(USART2_TX_IO),
#ifdef USART2_RX_REMAP_IO
    .rxChRemap = IO_TAG(USART2_RX_REMAP_IO),
#endif
#ifdef USART2_TX_REMAP_IO
    .txChRemap = IO_TAG(USART2_TX_REMAP_IO),
#endif
    .remap = GPIO_Remap_USART2,
};
#endif

#ifdef USE_USART3
static uartPort_t uartPort3;
static uint8_t uartPort3RxBuffer[UART3_RX_BUFFER_SIZE];
static uint8_t uartPort3TxBuffer[UART3_TX_BUFFER_SIZE];
static const uartHwDef_t uartPort3Def = {
    .uartPort = &uartPort3,
    .rxBuffer = uartPort3RxBuffer,
    .rxBufferSize = sizeof(uartPort3RxBuffer),
    .txBuffer = uartPort3TxBuffer,
    .txBufferSize = sizeof(uartPort3TxBuffer),
    .USARTx = USART3,
    .IRQn = USART3_IRQn,
    .IRQPrio = NVIC_PRIO_SERIALUART3,
    .rcc = RCC_APB1(USART3),
    .rxCh = IO_TAG(USART3_RX_IO),
    .txCh = IO_TAG(USART3_TX_IO),
#ifdef USART3_RX_PIN_REMAP_IO
    .rxChRemap = IO_TAG(USART3_RX_REMAP_IO),
#endif
#ifdef USART3_TX_PIN_REMAP_IO
    .txChRemap = IO_TAG(USART3_TX_REMAP_IO),
#endif
    .remap = GPIO_PartialRemap_USART3,
};
#endif

void serialUSARTHwInit(uartPort_t *self, const serialPortMode_t *config)
{
    UNUSED(config);
    const uartHwDef_t *def = self->hwDef;


    self->txDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;
    self->rxDMAPeripheralBaseAddr = (uint32_t)&self->USARTx->DR;


    RCC_ClockCmd(def->rcc, ENABLE);
}

void usartHwConfigurePins(uartPort_t *self, const serialPortMode_t *config) {
    ioRec_t *tx, *rx, *rxi, *txi;
    if(config->mode & MODE_U_REMAP) {
        rx = IOGetByTag(self->hwDef->rxChRemap);
        tx = IOGetByTag(self->hwDef->txChRemap);
        rxi = IOGetByTag(self->hwDef->rxCh);
        txi = IOGetByTag(self->hwDef->txCh);
        GPIO_PinRemapConfig(self->hwDef->remap, ENABLE);
        self->port.mode |= MODE_U_REMAP;
    } else {
        rx = IOGetByTag(self->hwDef->rxCh);
        tx = IOGetByTag(self->hwDef->txCh);
        rxi = IOGetByTag(self->hwDef->rxChRemap);
        txi = IOGetByTag(self->hwDef->txChRemap);
        GPIO_PinRemapConfig(self->hwDef->remap, DISABLE);
        self->port.mode &= ~MODE_U_REMAP;
    }

    if(self->port.mode & MODE_SINGLEWIRE) {
        IOConfigGPIO(tx, IOCFG_AF_OD);
    } else {
        if (self->port.mode & MODE_TX && tx)
            IOConfigGPIO(tx, IOCFG_AF_PP);   // TODO!
        if (self->port.mode & MODE_RX && rx)
            IOConfigGPIO(rx, IOCFG_IPU);
        if (txi)
            IOConfigGPIO(rxi, IOCFG_IPU);
        if (rxi)
            IOConfigGPIO(txi, IOCFG_IPU);
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
void USART2_IRQHandler(void)
{
    uartIrqHandler(&uartPort2);
}
#endif

#ifdef USE_USART3
void USART3_IRQHandler(void)
{
    uartIrqHandler(&uartPort3);
}
#endif
