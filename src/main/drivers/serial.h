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
typedef enum portMode_t {
    MODE_RX           = 1 << 0,
    MODE_TX           = 1 << 1,
    MODE_RXTX         = MODE_RX | MODE_TX,
    MODE_SBUS         = 1 << 2,
    MODE_HALFDUPLEX   =   1 << 3,
    MODE_SINGLEWIRE   = 1 << 4,
    MODE_INVERTED     = 1 << 5,
// softserial specific
    MODE_S_DUALTIMER  = 1 << 6, // try to claim adjacent timer channel in softserial mode
// uart specific
    MODE_U_DMARX      = 1 << 7, // use USART RX DMA if available
    MODE_U_DMATX      = 1 << 8, // use USART TX DMA if available
} portMode_t;



// port state is used to indicate (and change) state of open port.
typedef enum {
    // port direction
    STATE_TX             = 1 << 0,
    STATE_RX             = 1 << 1,
    STATE_RXTX           = STATE_RX|STATE_TX,
    // function flags
    STATE_RX_WHENTXDONE  = 1 << 2,  // TODO - implemented only for softserial
    STATE_TX_DELAY       = 1 << 3,  // TODO - not implemented
} portState_t;

// operations that can be invoked on serial port
// some of theese oparations are wrapped with serialXxx function
typedef enum {
    CMD_CONFIGURE,     // (const serialPortConfig_t* config)
    CMD_GET_CONFIG,    // (serialPortConfig_t* config)
    CMD_RELEASE,       // (void)
    CMD_ENABLE_STATE,  // (portState_t)
    CMD_DISABLE_STATE, // (portState_t)
    CMD_SET_DIRECTION, // (portState_t, only TX/RX)
    
} portCommand_t;

typedef void serialReceiveCallback(uint16_t data);   // used by serial drivers to return frames to app

typedef struct serialPort {

    const struct serialPortVTable *vTable;

    uint8_t identifier;
    portMode_t mode;
    portState_t state;
    
    uint32_t baudRate;

    uint32_t rxBufferSize;              // must be power of two
    uint32_t txBufferSize;              // must be power of two
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    serialReceiveCallback *rxCallback;
} serialPort_t;

// this structure holds all serial port configuration (GET_CONFIG/RELEASE/CONFIGURE must work)
typedef struct  {
    portMode_t mode;
    uint32_t baudRate;
    serialReceiveCallback *rxCallback;
} serialPortConfig_t;

// use this to initialize structure used to store port config. CMD_CONFIGURE can be safely called with it 
#define SERIAL_CONFIG_INIT_EMPTY { .mode=0 }

struct serialPortVTable {
    bool (*isSerialTransmitBufferEmpty)(serialPort_t *instance);
    void (*serialWriteByte)(serialPort_t *instance, uint8_t ch);

    int (*serialTotalBytesWaiting)(serialPort_t *instance);
    int (*serialReadByte)(serialPort_t *instance);
    
    int (*command)(serialPort_t *instance, portCommand_t cmd, void* data);
};

bool isSerialTransmitBufferEmpty(serialPort_t *instance);
void serialWrite(serialPort_t *instance, uint8_t ch);
//void serialWrite(serialPort_t *instance, uint8_t *data, int len);
void serialPrint(serialPort_t *instance, const char *str);

int serialTotalBytesWaiting(serialPort_t *instance);
int serialRead(serialPort_t *instance);
// store current configuration into passed struct if not null, release port
void serialRelease(serialPort_t *instance, serialPortConfig_t* config);
// restore previous configuration
void serialConfigure(serialPort_t *instance, const serialPortConfig_t* config);
// get actual configuration
void serialGetConfig(serialPort_t *instance, serialPortConfig_t* config);
void serialSetDirection(serialPort_t *instance, portState_t state);
void serialEnableState(serialPort_t *instance, portState_t state);
void serialDisableState(serialPort_t *instance, portState_t state);
int serialCmd(serialPort_t *instance, portCommand_t cmd, void* data);
