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

void serialPutc(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->putc(instance, ch);
}

void serialWrite(serialPort_t *instance, uint8_t ch) {
    serialPutc(instance, ch);
}

int serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->totalBytesWaiting(instance);
}

int serialGetc(serialPort_t *instance)
{
    return instance->vTable->getc(instance);
}

int serialRead(serialPort_t *instance)
{
    return serialGetc(instance);
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isTransmitBufferEmpty(instance);
}

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialPutc(instance, ch);
    }
}

void serialRelease(serialPort_t *instance)
{
    instance->vTable->release(instance);
}

void serialConfigure(serialPort_t *instance, const serialPortConfig_t* config)
{
    instance->vTable->configure(instance, config);
}

void serialGetConfig(serialPort_t *instance, serialPortConfig_t* config)
{
    instance->vTable->getConfig(instance, config);
}

void serialUpdateState(serialPort_t *instance, portState_t keepMask, portState_t setMask)
{
    instance->vTable->updateState(instance, keepMask, setMask);
}

void serialSetDirection(serialPort_t *instance, portState_t state)
{
    serialUpdateState(instance, ~STATE_RXTX, state);
}

