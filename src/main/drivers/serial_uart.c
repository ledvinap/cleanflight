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
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "common/utils.h"
#include "gpio.h"
#include "inverter.h"
#include "nvic.h"

#include "io.h" 

#include "timer.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

static void uartReconfigureState(uartPort_t *self);
extern const struct serialPortVTable uartVTable;

static void usartConfigurePinInversion(uartPort_t *self) {
#ifdef INVERTER
    if (self->USARTx == INVERTER_USART) {
        // Enable hardware inverter if available.
        if(self->port.mode & MODE_INVERTED)
            INVERTER_ON;
        else
            INVERTER_OFF;
    }
#endif

#ifdef STM32F303xC
    uint32_t inversionPins = 0;

    if (self->port.mode & MODE_TX) {
        inversionPins |= USART_InvPin_Tx;
    }
    if (self->port.mode & MODE_RX) {
        inversionPins |= USART_InvPin_Rx;
    }

    USART_InvPinCmd(self->USARTx, inversionPins, uartPort->port.mode & MODE_INVERTED ? ENABLE : DISABLE);
#endif

}

static void usartConfigurePins(uartPort_t *self, const serialPortConfig_t *config) {
    IOId_t tx,rx,rxi,txi;
    if(config->mode & MODE_U_REMAP) {
        rx = self->hwDef->rxChRemap;
        tx = self->hwDef->txChRemap;
        rxi = self->hwDef->rxCh;
        txi = self->hwDef->txCh;
        GPIO_PinRemapConfig(self->hwDef->remap, ENABLE);
        self->port.mode |= MODE_U_REMAP;
    } else {
        rx = self->hwDef->rxCh;
        tx = self->hwDef->txCh;
        rxi = self->hwDef->rxChRemap;
        txi = self->hwDef->txChRemap;
        GPIO_PinRemapConfig(self->hwDef->remap, DISABLE);
        self->port.mode &= ~MODE_U_REMAP;
    }

    if(self->port.mode & MODE_SINGLEWIRE) {
        timerChConfigGPIO(&timerHardware[tx], Mode_AF_OD);
    } else {
        if (self->port.mode & MODE_TX && tx != IO_NONE)
            timerChConfigGPIO(getIOHw(tx), Mode_AF_PP);   // TODO!
        if (self->port.mode & MODE_RX && rx != IO_NONE)
            timerChConfigGPIO(getIOHw(rx), Mode_IPU);
        if (txi != IO_NONE)
             timerChConfigGPIO(getIOHw(rxi), Mode_IPU);
        if (rxi != IO_NONE)
             timerChConfigGPIO(getIOHw(txi), Mode_IPU);
    }
}

static void usartConfigureDMAorIRQ(uartPort_t *self, const serialPortConfig_t *config)
{
    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    self->port.mode &= ~(MODE_U_DMATX | MODE_U_DMARX);

    if (self->port.mode & MODE_RX) {
        if (self->rxDMAChannel && config->mode & MODE_U_DMARX) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr = self->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

            DMA_InitStructure.DMA_BufferSize = self->port.rxBufferSize;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)self->port.rxBuffer;
            DMA_DeInit(self->rxDMAChannel);
            DMA_Init(self->rxDMAChannel, &DMA_InitStructure);
            DMA_Cmd(self->rxDMAChannel, ENABLE);
            USART_DMACmd(self->USARTx, USART_DMAReq_Rx, ENABLE);
            self->rxDMAPos = DMA_GetCurrDataCounter(self->rxDMAChannel);

            self->port.mode |= MODE_U_DMARX;
        } else {
            NVIC_InitTypeDef NVIC_InitStructure;
            NVIC_InitStructure.NVIC_IRQChannel = self->hwDef->IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(self->hwDef->IRQPrio);
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(self->hwDef->IRQPrio);
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);

            USART_ClearITPendingBit(self->USARTx, USART_IT_RXNE);
            USART_ITConfig(self->USARTx, USART_IT_RXNE, ENABLE);
        }
    }

    // Transmit DMA or IRQ
    if (self->port.mode & MODE_TX) {
        if (self->txDMAChannel && config->mode & MODE_U_DMATX) {
            DMAInitNVIC(self->hwDef->txDMAChannelId, self->hwDef->IRQPrio_txDMA);

            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr = self->txDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

            DMA_InitStructure.DMA_BufferSize = self->port.txBufferSize;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_DeInit(self->txDMAChannel);
            DMA_Init(self->txDMAChannel, &DMA_InitStructure);
            DMA_ITConfig(self->txDMAChannel, DMA_IT_TC, ENABLE);
            DMA_SetCurrDataCounter(self->txDMAChannel, 0);
            self->txDMAChannel->CNDTR = 0;
            USART_DMACmd(self->USARTx, USART_DMAReq_Tx, ENABLE);

            self->port.mode |= MODE_U_DMATX;
        } else {
            NVIC_InitTypeDef NVIC_InitStructure;
            NVIC_InitStructure.NVIC_IRQChannel = self->hwDef->IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(self->hwDef->IRQPrio);
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(self->hwDef->IRQPrio);
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);

            USART_ITConfig(self->USARTx, USART_IT_TXE, ENABLE);
        }
    }
}

static void uartReconfigure(uartPort_t *self, const serialPortConfig_t *config)
{
    self->txDMAEmpty = true;
    // common serial initialisation code should move to serialPort::init()
    self->port.rxBufferHead = self->port.rxBufferTail = 0;
    self->port.txBufferHead = self->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    self->port.rxCallback = config->rxCallback;

    // setup initial port state
    self->port.mode = 0;
    self->port.state = 0;
    if(config->mode & MODE_RX) {
        self->port.state |= STATE_RX;
        self->port.mode |= MODE_RX;
    }
    if(config->mode & MODE_TX) {
        self->port.state |= STATE_TX;
        self->port.mode |= MODE_TX;
    }
    self->port.mode |= config->mode & MODE_SINGLEWIRE;
    self->port.baudRate = config->baudRate;

    usartConfigurePins(self, config);
    usartConfigureDMAorIRQ(self, config);

    uartReconfigureState(self);
}


static void uartReconfigureState(uartPort_t *self)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(self->USARTx, DISABLE);

    USART_InitStructure.USART_BaudRate = self->port.baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    if (self->port.mode & MODE_SBUS) {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    } else {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (self->port.state & STATE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (self->port.state & STATE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;

    USART_Init(self->USARTx, &USART_InitStructure);
    usartConfigurePinInversion(self);
    if (self->port.mode & MODE_SINGLEWIRE)
        USART_HalfDuplexCmd(self->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(self->USARTx, DISABLE);
    USART_Cmd(self->USARTx, ENABLE);
}

serialPort_t *uartOpen(USART_TypeDef *USARTx, const serialPortConfig_t *config)
{
    const uartHwDef_t* def;
    def = serialUSARTFindDef(USARTx);
    if(!def) return NULL;

    uartPort_t *self=def->uartPort;

    self->port.vTable = &uartVTable;

    self->port.rxBuffer = def->rxBuffer;
    self->port.txBuffer = def->txBuffer;
    self->port.rxBufferSize = def->rxBufferSize;
    self->port.txBufferSize = def->txBufferSize;

    self->hwDef = def;
    self->USARTx = def->USARTx;

    self->rxDMAChannel = DMAGetChannel(def->rxDMAChannelId);
    self->txDMAChannel = DMAGetChannel(def->txDMAChannelId);

    serialUSARTHwInit(self, config);
    uartReconfigure(self, config);
    return &self->port;
}

// this function will need critical section if extended serial functions are implemented
void uartUpdateState(serialPort_t *serial, portState_t andMask, portState_t orMask)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    portState_t newState = (self->port.state & andMask) | orMask;
    self->port.state = newState;
    uartReconfigureState(self);
}

void uartConfigure(serialPort_t *serial, const serialPortConfig_t *config)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    // just call reconfigure now. keep this in sync with uartRelease
    // TODO - we should reaquire DMA channels
    if(config->mode == 0)  // check for dummy config
        return;

    uartReconfigure(self, config);
}

void uartRelease(serialPort_t *serial)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    uartUpdateState(&self->port, 0, 0);
    // DMA channels should be released
    USART_Cmd(self->USARTx, DISABLE);
    self->port.mode = 0;
}

void uartGetConfig(serialPort_t *serial, serialPortConfig_t* config)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);

    config->baudRate = self->port.baudRate;  // TODO - use actual baudrate
    config->mode = self->port.mode;
    config->rxCallback = self->port.rxCallback;
}

void uartStartTxDMA(uartPort_t *self)
{
    self->txDMAChannel->CMAR = (uint32_t)&self->port.txBuffer[self->port.txBufferTail];
    // TODO - data passed to DMA transfer are 'released' from queue immediately and could be overwritten. txBufferTail should be moved only after transfer is complete
    //  but beware that whole queue may be pending then. half interrupt can help
    // usart_write does not check buffer space anyway now ...
    if (self->port.txBufferHead > self->port.txBufferTail) {
        self->txDMAChannel->CNDTR = self->port.txBufferHead - self->port.txBufferTail;
        self->port.txBufferTail = self->port.txBufferHead;
    } else {
        self->txDMAChannel->CNDTR = self->port.txBufferSize - self->port.txBufferTail;
        self->port.txBufferTail = 0;
    }
    self->txDMAEmpty = false;
    DMA_Cmd(self->txDMAChannel, ENABLE);
}

void uartTxDMAHandler(uartPort_t *self)
{
    if (self->port.txBufferHead != self->port.txBufferTail)
        uartStartTxDMA(self);
    else
        self->txDMAEmpty = true;
}

void uartIrqHandler(uartPort_t *self)
{
#if defined(STM32F10X)
    uint16_t flags = self->USARTx->SR & self->USARTx->CR1;
#elif defined(STM32F303)
    uint32_t flags = self->USARTx->ISR;
#else
# error "Unknown CPU"
#endif

    if (!(self->port.mode & MODE_U_DMARX) && (flags & USART_FLAG_RXNE)) {
        if (self->port.rxCallback) {
            self->port.rxCallback(USART_ReceiveData(self->USARTx));
        } else {
            self->port.rxBuffer[self->port.rxBufferHead] = USART_ReceiveData(self->USARTx);
            self->port.rxBufferHead = (self->port.rxBufferHead + 1 >= self->port.rxBufferSize) ? 0 : self->port.rxBufferHead + 1;
        }
    }

    if (!(self->port.mode & MODE_U_DMATX) && (flags & USART_FLAG_TXE)) {
        if (self->port.txBufferTail != self->port.txBufferHead) {
            USART_SendData(self->USARTx, self->port.txBuffer[self->port.txBufferTail]);
            self->port.txBufferTail = (self->port.txBufferTail + 1 >= self->port.txBufferSize) ? 0 : self->port.txBufferTail + 1;
        } else {
            USART_ITConfig(self->USARTx, USART_IT_TXE, DISABLE);
        }
    }
#ifdef STM32F303
    // TODO - is this really neccesary?
    if (flags & USART_FLAG_ORE)
    {
        USART_ClearITPendingBit (self->USARTx, USART_IT_ORE);
    }
#endif
}

// interface implemenatition

bool isUartTransmitBufferEmpty(serialPort_t *serial)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    if (self->port.mode & MODE_U_DMATX)
        return self->txDMAEmpty;
    else
        return self->port.txBufferTail == self->port.txBufferHead;
}

void uartWrite(serialPort_t *serial, uint8_t ch)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    // TODO - check for full buffer

    self->port.txBuffer[self->port.txBufferHead] = ch;
    self->port.txBufferHead = (self->port.txBufferHead + 1 >= self->port.txBufferSize) ? 0 : self->port.txBufferHead + 1;

    if (self->port.mode & MODE_U_DMATX) {
        if (!(self->txDMAChannel->CCR & 1))
            uartStartTxDMA(self);
    } else {
        USART_ITConfig(self->USARTx, USART_IT_TXE, ENABLE);
    }
}

int uartTotalBytesWaiting(serialPort_t *serial)
{
    uartPort_t *self = container_of(serial, uartPort_t, port);
    int ret;
    if (self->port.mode & MODE_U_DMARX) {
        ret = self->rxDMAChannel->CNDTR - self->rxDMAPos;
    } else {
        ret = self->port.rxBufferHead - self->port.rxBufferTail;
    }
    if(ret < 0)
        ret += self->port.rxBufferSize;
    return ret;
}

int uartRead(serialPort_t *serial)
{
    uint8_t ch;
    uartPort_t *self = container_of(serial, uartPort_t, port);

    // TODO - this function shoud check for empty buffer

    if (self->port.mode & MODE_U_DMARX) {
        ch = self->port.rxBuffer[self->port.rxBufferSize - self->rxDMAPos];
        if (self->rxDMAPos == 1)
            self->rxDMAPos = self->port.rxBufferSize;
        else
            self->rxDMAPos --;
    } else {
        ch = self->port.rxBuffer[self->port.rxBufferTail];
        self->port.rxBufferTail = (self->port.rxBufferTail + 1 >= self->port.rxBufferSize) ? 0 : self->port.rxBufferTail + 1;
    }

    return ch;
}

const struct serialPortVTable uartVTable = {
    .isTransmitBufferEmpty = isUartTransmitBufferEmpty,
    .write = uartWrite,
    .totalBytesWaiting = uartTotalBytesWaiting,
    .read = uartRead,

    .release = uartRelease,
    .configure = uartConfigure,
    .getConfig = uartGetConfig,
    .updateState = uartUpdateState
};
