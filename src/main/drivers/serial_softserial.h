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

#include "timer_input.h"
#include "timer_output.h"
#include "timer_queue.h"

#define SOFT_SERIAL_BUFFER_SIZE 256

typedef enum {
    SOFTSERIAL1 = 0,
    SOFTSERIAL2
} softSerialPortIndex_e;

typedef struct softSerial_s {
    serialPort_t     port;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[SOFT_SERIAL_BUFFER_SIZE];

    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[SOFT_SERIAL_BUFFER_SIZE];

    uint32_t         bitTime;                             // length of bit time in timer ticks, 24.8 fixed point
    uint32_t         invBitTime;                          // inverse bit time, scaled to 16.16

    timerInputRec_t  rxTimerCh;
    timerQueueRec_t  rxTimerQ;
    callbackRec_t    rxCallback;
    uint16_t         rxStartRef;                            // timestamp of startbit edge
    uint16_t         rxInternalBuffer;                    // includes start and stop bits
    int8_t           rxBitIndex;                           

    timerOutputRec_t txTimerCh;
    callbackRec_t    txCallback;
    
    bool             directionTx;                         // current direction for singlewire mode 
    bool             directionRxOnDone;                   // switch to rx mode when all data are transmitted
    
    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;
} softSerial_t;

extern softSerial_t softSerialPorts[];

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, const serialPortConfig_t* config);

// serialPort API
void softSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t softSerialTotalBytesWaiting(serialPort_t *instance);
uint8_t softSerialReadByte(serialPort_t *instance);
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isSoftSerialTransmitBufferEmpty(serialPort_t *s);

