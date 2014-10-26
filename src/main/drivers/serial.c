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
#include <stdlib.h>

#include "platform.h"

#include "serial.h"

// TODO - implement optimized version that can write more bytes
void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWriteByte(instance, ch);
}

int serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalBytesWaiting(instance);
}

int serialRead(serialPort_t *instance)
{
    return instance->vTable->serialReadByte(instance);
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

void serialRelease(serialPort_t *instance, serialPortConfig_t* config)
{
    if(config)
        serialGetConfig(instance, config);
    serialCmd(instance, CMD_RELEASE, NULL);
}

void serialConfigure(serialPort_t *instance, const serialPortConfig_t* config)
{
    serialCmd(instance, CMD_CONFIGURE, (void*)config);
}

void serialGetConfig(serialPort_t *instance, serialPortConfig_t* config)
{
    serialCmd(instance, CMD_GET_CONFIG, (void*)config);
}

void serialSetDirection(serialPort_t *instance, portState_t state)
{
    serialCmd(instance, CMD_SET_DIRECTION, (void*)state);
}

void serialEnableState(serialPort_t *instance, portState_t state)
{
    serialCmd(instance, CMD_ENABLE_STATE, (void*)state);
}

void serialDisableState(serialPort_t *instance, portState_t state)
{
    serialCmd(instance, CMD_DISABLE_STATE, (void*)state);
}

int serialCmd(serialPort_t *instance, portCommand_t cmd, void* data)
{
    return instance->vTable->command(instance, cmd, data);
}
