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

// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.
#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    256
#define UART2_RX_BUFFER_SIZE    256
#define UART2_TX_BUFFER_SIZE    256
#define UART3_RX_BUFFER_SIZE    256
#define UART3_TX_BUFFER_SIZE    256

struct uartHwDef_s;

typedef struct {
    serialPort_t port;

    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;

//    uint32_t rxDMAIrq;
//    uint32_t txDMAIrq;

    uint32_t rxDMAPos;
    bool txDMAEmpty;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

    USART_TypeDef *USARTx;
    const struct uartHwDef_s* hwDef;
} uartPort_t;

serialPort_t *uartOpen(USART_TypeDef *USARTx, const serialPortMode_t *config);

// serialPort API
bool isUartTransmitBufferEmpty(serialPort_t *instance);
void uartWrite(serialPort_t *instance, uint8_t ch);
int uartTotalBytesWaiting(serialPort_t *instance);
int uartRead(serialPort_t *instance);

void uartRelease(serialPort_t *instance);
void uartConfigure(serialPort_t *instance, const serialPortMode_t* config);
void uartGetConfig(serialPort_t *instance, serialPortMode_t* config);
void uartUpdateState(serialPort_t *instance, portState_t keepMask, portState_t setMask);
