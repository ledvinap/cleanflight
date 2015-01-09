#pragma once

#include "dma.h"

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
    uint32_t APB1Periph, APB2Periph;
    IOId_t rxCh, txCh, rxChRemap, txChRemap;
    uint32_t remap;
} uartHwDef_t;

void serialUSARTHwInit(uartPort_t *self, const serialPortConfig_t *config);
const uartHwDef_t* serialUSARTFindDef(USART_TypeDef *USARTx);

void uartIrqHandler(uartPort_t *self);
void uartStartTxDMA(uartPort_t *self);
void uartTxDMAHandler(uartPort_t *self);
