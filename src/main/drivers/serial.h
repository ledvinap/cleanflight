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

// port mode is set when port is open and can't be changed (reopen port if neccesary)
// there are about 64 bits for customization, so it should be enough for some time
// change this to configuration structure (possibly with bitfields) if this is going out of control
typedef enum portMode_t {
    MODE_RX           = 1 << 0,
    MODE_TX           = 1 << 1,
    MODE_RXTX         = MODE_RX | MODE_TX,
    MODE_SBUS         = 1 << 2,
    MODE_SINGLEWIRE   = 1 << 3,
    MODE_HALFDUPLEX   = 1 << 4,
    MODE_INVERTED     = 1 << 5,
// driver specific modes below
// softserial specific
    MODE_S_DUALTIMER  = 1 << 6, // try to claim adjacent timer channel in softserial mode
// uart specific
    MODE_U_DMARX      = 1 << 7, // use USART RX DMA if available
    MODE_U_DMATX      = 1 << 8, // use USART TX DMA if available
    MODE_U_REMAP      = 1 << 9, // remap USART to alternate pins

// hints how to setup port if more configuration options are possible (DMA, dualtimer, .. ). Device-specific driver sets neccesary flags
    MODE_DEFAULT_FAST  = MODE_U_DMARX | MODE_U_DMATX, // setup port for high-performance if possible
} portMode_t;



// port state is used to indicate (and change) state of open port
// some port functions can be triggered by bit in this field
typedef enum {
    // port direction
    STATE_RX             = 1 << 0,
    STATE_TX             = 1 << 1,
    STATE_RXTX           = STATE_RX | STATE_TX,
    // function flags
    STATE_RX_WHENTXDONE  = 1 << 2,  // TODO - implemented only for softserial
    STATE_TX_DELAY       = 1 << 3,  // TODO - not implemented
} portState_t;

typedef void serialReceiveCallback(uint16_t data);   // used by serial drivers to return frames to app

typedef struct serialPort {
    const struct serialPortVTable *vTable;

    uint8_t identifier;
    portMode_t mode;
    portState_t state;

    uint32_t baudRate;

    uint32_t rxBufferSize;              // must be power of two
    uint32_t txBufferSize;              // must be power of two
    uint8_t *rxBuffer;
    uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    serialReceiveCallback *rxCallback;
} serialPort_t;

// this structure holds all serial port configuration (GetConfig/Release/Configure must return port to functional state)
typedef struct  {
    portMode_t mode;
    uint32_t baudRate;
    serialReceiveCallback *rxCallback;
    uint8_t rxPin, txPin;
} serialPortMode_t;

// use this to initialize structure used to store port config. serialConfigure can be safely called with it
#define SERIAL_CONFIG_INIT_EMPTY { .mode = 0 }

struct serialPortVTable {
    bool (*isTransmitBufferEmpty)(serialPort_t *instance);
    void (*write)(serialPort_t *instance, uint8_t ch);

    int (*totalBytesWaiting)(serialPort_t *instance);
    int (*read)(serialPort_t *instance);

    void (*release)(serialPort_t *instance);
    void (*configure)(serialPort_t *instance, const serialPortMode_t* config);
    void (*getConfig)(serialPort_t *instance, serialPortMode_t* config);
    void (*updateState)(serialPort_t *serial, portState_t keepMask, portState_t setMask);
};

bool isSerialTransmitBufferEmpty(serialPort_t *instance);
void serialWrite(serialPort_t *instance, uint8_t ch);
int serialWriteData(serialPort_t *instance, const uint8_t *data, int len);  // for backward comaptibility
int serialWriteBlock(serialPort_t *instance, const uint8_t *data, int len);
void serialPrint(serialPort_t *instance, const char *str);

int serialTotalBytesWaiting(serialPort_t *instance);
int serialRead(serialPort_t *instance);
int serialReadBlock(serialPort_t *instance, const uint8_t *data, int len);
// release port
void serialRelease(serialPort_t *instance);
// restore previous configuration
void serialConfigure(serialPort_t *instance, const serialPortMode_t* config);
// get actual configuration
void serialGetConfig(serialPort_t *instance, serialPortMode_t* config);
// change serial state, bits not in keepMask are reset(&), then bits in setmask are set (|)
void serialUpdateState(serialPort_t *serial, portState_t keepMask, portState_t setMask);

// convenience wrapper to serialUpdateState. Not implemented by vTable
void serialSetDirection(serialPort_t *instance, portState_t state);
