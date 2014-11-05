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

// FIXME since serial ports can be used for any function these buffer sizes probably need normalising.
// this should be probably moved to target directory
// buffer size must be power of 2

#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    256
#define UART2_RX_BUFFER_SIZE    128
#define UART2_TX_BUFFER_SIZE    64
#define UART3_RX_BUFFER_SIZE    128
#define UART3_TX_BUFFER_SIZE    64

typedef struct {
    serialPort_t port;

    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;

    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;
    bool txDMAEmpty;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

    USART_TypeDef *USARTx;
} uartPort_t;

serialPort_t *uartOpen(USART_TypeDef *USARTx, const serialPortConfig_t *config);

// serialPort API
bool isUartTransmitBufferEmpty(serialPort_t *instance);
void uartPutc(serialPort_t *instance, uint8_t ch);
int uartTotalBytesWaiting(serialPort_t *instance);
int uartGetc(serialPort_t *instance);

void uartRelease(serialPort_t *instance);
void uartConfigure(serialPort_t *instance, const serialPortConfig_t* config);
void uartGetConfig(serialPort_t *instance, serialPortConfig_t* config);
void uartUpdateState(serialPort_t *instance, portState_t keepMask, portState_t setMask);
