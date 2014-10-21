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

// port mode is set when port is open and should not change (reopen port if neccesary)
typedef enum portMode_t {
    MODE_RX         = 1 << 0,
    MODE_TX         = 1 << 1,
    MODE_RXTX       = MODE_RX | MODE_TX,
    MODE_SBUS       = 1 << 2,
    MODE_HALFDUPLEX = 1 << 3,
    MODE_SINGLEWIRE = 1 << 4,
    MODE_INVERTED   = 1 << 5,
} portMode_t;

// port state is used to change state of open port.
// TODO - this needs some work, interface is a bit confusing now
typedef enum portState_t {
    STATE_TX             = 1 << 0,
    STATE_RX             = 1 << 1,
    STATE_RXTX           = STATE_RX|STATE_TX,
    STATE_RX_WHENTXDONE  = 1 << 2,  // TODO - implemented only for softserial
    STATE_TX_DELAY       = 1 << 3,  // TODO - not implemented

    STATE_CMD_SET        = 1 << 7,
    STATE_CMD_CLEAR      = 1 << 8
} portState_t;

typedef enum portConfigOperation_t {
    OP_CONFIGURE,
    OP_GET_CONFIG,
    OP_RELEASE,
} portConfigOperation_t;

typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app

typedef struct serialPort {

    const struct serialPortVTable *vTable;

    uint8_t identifier;
    portMode_t mode;
    portState_t state;
    
    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    // FIXME rename member to rxCallback
    serialReceiveCallbackPtr rxCallback;
} serialPort_t;

// this structure should hold all info necessary for serial port initialization
// is is currently only used to save port state when changing function,
// but tighter integration would be nice
typedef struct  {
    portMode_t mode;
    uint32_t baudRate;
    serialReceiveCallbackPtr rxCallback;
} serialPortConfig_t;

#define SERIAL_CONFIG_INIT_EMPTY { .mode=0 }

struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);
    uint8_t (*serialTotalBytesWaiting)(serialPort_t *instance);
    uint8_t (*serialRead)(serialPort_t *instance);
    bool (*isSerialTransmitBufferEmpty)(serialPort_t *instance);
    void (*setState)(serialPort_t *instance, portState_t state);
    void (*configure)(serialPort_t *instance, portConfigOperation_t op, serialPortConfig_t* config);
};

void serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t serialTotalBytesWaiting(serialPort_t *instance);
uint8_t serialRead(serialPort_t *instance);
void serialSetState(serialPort_t *instance, portState_t state);
bool isSerialTransmitBufferEmpty(serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
// store current configuration into passed struct if not null, release port
void serialRelease(serialPort_t *instance, serialPortConfig_t* config);
// restore previous configuration
void serialConfigure(serialPort_t *instance, const serialPortConfig_t* config);
// get actual configuration
void serialGetConfig(serialPort_t *instance, serialPortConfig_t* config);
