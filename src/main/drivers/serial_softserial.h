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

#define SOFTSERIAL_BUFFER_SIZE 256

typedef enum {
    SOFTSERIAL1 = 0,
    SOFTSERIAL2
} softSerialPortIndex_e;

typedef struct softSerial_s {
    serialPort_t     port;

    const timerHardware_t *rxTimerHardware;
    volatile uint8_t rxBuffer[SOFTSERIAL_BUFFER_SIZE];

    const timerHardware_t *txTimerHardware;
    volatile uint8_t txBuffer[SOFTSERIAL_BUFFER_SIZE];

    uint32_t         bitTime;                             // length of bit time in timer ticks, 24.8 fixed point
    uint32_t         invBitTime;                          // inverse bit time, scaled to 16.16
    uint16_t         symbolLength;                        // length of whole symbol (start+parity+stop-0.5)

    timerInputRec_t  rxTimerCh;
    timerQueueRec_t  rxTimerQ;
    callbackRec_t    rxCallback;

    timerOutputRec_t txTimerCh;
    callbackRec_t    txCallback;

    bool             directionRxOnDone;                   // switch to rx mode when all data are transmitted

    uint16_t         transmissionErrors;
    uint16_t         receiveErrors;
} softSerial_t;

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, const serialPortConfig_t* config);

// serialPort API
bool isSoftSerialTransmitBufferEmpty(serialPort_t *instance);
void softSerialPutc(serialPort_t *instance, uint8_t ch);
int softSerialTotalBytesWaiting(serialPort_t *instance);
int softSerialGetc(serialPort_t *instance);

void softSerialRelease(serialPort_t *instance);
void softSerialConfigure(serialPort_t *instance, const serialPortConfig_t* config);
void softSerialGetConfig(serialPort_t *instance, serialPortConfig_t* config);
void softSerialUpdateState(serialPort_t *instance, portState_t keepMask, portState_t setMask);

