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

#include <platform.h>

#include "build_config.h"

#include "common/utils.h"
#include "gpio.h"
#include "inverter.h"
#include "nvic.h"

#include "io.h"

#include "timer.h"

#include "serial.h"
#include "serial_impl.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"
#ifdef STM32F10X
#include "serial_uart_stm32f10x.h"
#endif
#ifdef STM32F303xC
#include "serial_uart_stm32f30x.h"
#endif

void usartInitAllIOSignals(void)
{
#ifdef STM32F10X
    // Set UART1 TX to output and high state to prevent a rs232 break condition on reset.
    // See issue https://github.com/cleanflight/cleanflight/issues/1433
    gpio_config_t gpio;

    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpio.pin = UART1_TX_PIN;
    digitalHi(UART1_GPIO, gpio.pin);
    gpioInit(UART1_GPIO, &gpio);

    // Set TX of UART2 and UART3 to input with pull-up to prevent floating TX outputs.
    gpio.mode = Mode_IPU;

#ifdef USE_UART2
    gpio.pin = UART2_TX_PIN;
    gpioInit(UART2_GPIO, &gpio);
#endif

#ifdef USE_UART3
    gpio.pin = UART3_TX_PIN;
    gpioInit(UART3_GPIO, &gpio);
#endif

#endif

#ifdef STM32F303
    // Set TX for UART1, UART2 and UART3 to input with pull-up to prevent floating TX outputs.
    gpio_config_t gpio;

    gpio.mode = Mode_IPU;
    gpio.speed = Speed_2MHz;

#ifdef USE_UART1
    gpio.pin = UART1_TX_PIN;
    gpioInit(UART1_GPIO, &gpio);
#endif

//#ifdef USE_UART2
//    gpio.pin = UART2_TX_PIN;
//    gpioInit(UART2_GPIO, &gpio);
//#endif

#ifdef USE_UART3
    gpio.pin = UART3_TX_PIN;
    gpioInit(UART3_GPIO, &gpio);
#endif

#endif
}

static void uartReconfigureState(uartPort_t *self);
extern const struct serialPortVTable uartVTable;

static void usartConfigurePinInversion(uartPort_t *self) {
#if !defined(INVERTER) && !defined(STM32F303xC)
    UNUSED(self);
#endif

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

    USART_InvPinCmd(self->USARTx, inversionPins, (self->port.mode & MODE_INVERTED) ? ENABLE : DISABLE);
#endif
}


static void usartConfigureDMAorIRQ(uartPort_t *self, const serialPortMode_t *config)
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

static void uartReconfigure(uartPort_t *self, const serialPortMode_t *config)
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
    self->port.mode |= config->mode & (MODE_SINGLEWIRE | MODE_INVERTED);
    self->port.baudRate = config->baudRate;

    usartHwConfigurePins(self, config);
    usartConfigurePinInversion(self);
    usartConfigureDMAorIRQ(self, config);

    uartReconfigureState(self);
}


static void uartReconfigureState(uartPort_t *self)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(self->USARTx, DISABLE);

    USART_InitStructure.USART_BaudRate = self->port.baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = (self->port.mode & MODE_STOPBITS2) ? USART_StopBits_1 : USART_StopBits_1;
    USART_InitStructure.USART_Parity = (self->port.mode & MODE_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (self->port.state & STATE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (self->port.state & STATE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
// TODO - check inversion and other flags to be copied ... 
    USART_Init(self->USARTx, &USART_InitStructure);
    usartConfigurePinInversion(self);
    if (self->port.mode & MODE_SINGLEWIRE)
        USART_HalfDuplexCmd(self->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(self->USARTx, DISABLE);
    USART_Cmd(self->USARTx, ENABLE);
}

serialPort_t *uartOpen(USART_TypeDef *USARTx, const serialPortMode_t *config)
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
    if(orMask & STATE_RX_WHENTXDONE) {  // hardware will handle this 
        orMask |= STATE_RX;
        andMask &= STATE_RX;
    }
    portState_t newState = (self->port.state & andMask) | orMask;
    self->port.state = newState;
    uartReconfigureState(self);
}

void uartConfigure(serialPort_t *serial, const serialPortMode_t *config)
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
    // stop DMA, disable interrupts
    if (self->port.mode & MODE_U_DMARX)
        DMA_DeInit(self->rxDMAChannel);
    if(self->port.mode & MODE_U_DMATX)
        DMA_DeInit(self->txDMAChannel);
    USART_ITConfig(self->USARTx, USART_IT_TXE | USART_IT_RXNE, DISABLE);
    USART_Cmd(self->USARTx, DISABLE);
    self->port.mode = 0;
}

void uartGetConfig(serialPort_t *serial, serialPortMode_t* config)
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
#elif defined(STM32F303xC)
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

    if (flags & USART_FLAG_TXE) {
        if (!(self->port.mode & MODE_U_DMATX) && self->port.txBufferTail != self->port.txBufferHead) {
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

int uartTxBytesFree(serialPort_t *instance)
{
    int bytesFree = serialTxBytesFree_Generic(instance);

    uartPort_t *self = container_of(instance, uartPort_t, port);

    if (self->port.mode & MODE_U_DMATX) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must subtract
         * the remaining size of that in-progress transfer here instead:
         */
        bytesFree -= self->txDMAChannel->CNDTR;

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesFree < 0) {
            return 0;
        }
    }

    return bytesFree;
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

int uartRxBytesWaiting(serialPort_t *serial)
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
    .txBytesFree = uartTxBytesFree,
    .write = uartWrite,
    .rxBytesWaiting = uartRxBytesWaiting,
    .read = uartRead,

    .release = uartRelease,
    .configure = uartConfigure,
    .getConfig = uartGetConfig,
    .updateState = uartUpdateState
};
