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

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
// device specific uart implementation is defined here


typedef struct uartHwDef_s {
    uartPort_t *uartPort;
    uint8_t *rxBuffer; int rxBufferSize;
    uint8_t *txBuffer; int txBufferSize;
    USART_TypeDef *USARTx;
    uint8_t IRQn;
    uint8_t IRQPrio;
    DMAChannelID rxDMAChannelId, txDMAChannelId;
    uint8_t IRQPrio_rxDMA, IRQPrio_txDMA;
    rccPeriphTag_t rcc; 
    const ioDef_t *rxCh, *txCh, *rxChRemap, *txChRemap;
    uint8_t afConfig, afConfigRemap;
    uint32_t remap;
} uartHwDef_t;

void serialUSARTHwInit(uartPort_t *self, const serialPortMode_t *config);
const uartHwDef_t* serialUSARTFindDef(USART_TypeDef *USARTx);

void usartHwConfigurePins(uartPort_t *self, const serialPortMode_t *config);
void uartIrqHandler(uartPort_t *self);
void uartStartTxDMA(uartPort_t *self);
void uartTxDMAHandler(uartPort_t *self);
