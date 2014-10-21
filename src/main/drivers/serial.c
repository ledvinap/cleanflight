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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "serial.h"

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

uint32_t serialGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

uint8_t serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalBytesWaiting(instance);
}

uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetState(serialPort_t *instance, portState_t state)
{
    instance->vTable->setState(instance, state);
}

void serialRelease(serialPort_t *instance, serialPortConfig_t* config)
{
    if(config)
        instance->vTable->configure(instance, OP_GET_CONFIG, config);
    instance->vTable->configure(instance, OP_RELEASE, config);
}

void serialConfigure(serialPort_t *instance, const serialPortConfig_t* config)
{
    instance->vTable->configure(instance, OP_CONFIGURE, (serialPortConfig_t*)config);
}

void serialGetConfig(serialPort_t *instance, serialPortConfig_t* config)
{
    instance->vTable->configure(instance, OP_GET_CONFIG, config);
}

