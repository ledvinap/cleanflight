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

#include "serial.h"

#define SOFTSERIAL_BUFFER_SIZE 256

typedef enum {
    SOFTSERIAL1 = 0,
    SOFTSERIAL2
} softSerialPortIndex_e;

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, const serialPortMode_t* config);

// serialPort API
bool isSoftSerialTransmitBufferEmpty(serialPort_t *instance);
void softSerialWrite(serialPort_t *instance, uint8_t ch);
int softSerialTotalBytesWaiting(serialPort_t *instance);
int softSerialRead(serialPort_t *instance);

void softSerialRelease(serialPort_t *instance);
void softSerialConfigure(serialPort_t *instance, const serialPortMode_t* config);
void softSerialGetConfig(serialPort_t *instance, serialPortMode_t* config);
void softSerialUpdateState(serialPort_t *instance, portState_t keepMask, portState_t setMask);

