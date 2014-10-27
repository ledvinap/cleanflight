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

/*
 * telemetry_MSP.c
 *
 *  Created on: 22 Apr 2014
 *      Author: trey marc
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#ifdef TELEMETRY

#include "drivers/serial.h"
#include "telemetry/telemetry.h"
#include "io/serial_msp.h"
#include "io/serial.h"

static telemetryConfig_t *telemetryConfig;

#define MSP_TELEMETRY_BAUDRATE 19200 // TODO make this configurable

static serialPort_t *mspTelemetryPort;

static const serialPortConfig_t mspTelemetrySerialPortConfig = { .mode = MODE_TX|MODE_RX, .baudRate = MSP_TELEMETRY_BAUDRATE };  

void initMSPTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
}

void handleMSPTelemetry(void)
{
    sendMspTelemetry();
}

static serialPortConfig_t previousSerialConfig = SERIAL_CONFIG_INIT_EMPTY;
void freeMSPTelemetryPort(void)
{
    // FIXME only need to reset the port if the port is shared
    serialConfigure(mspTelemetryPort, &previousSerialConfig);
    endSerialPortFunction(mspTelemetryPort, FUNCTION_TELEMETRY);
}

void configureMSPTelemetryPort(void)
{
    mspTelemetryPort = findOpenSerialPort(FUNCTION_TELEMETRY);
    if (mspTelemetryPort) {
        serialGetConfig(mspTelemetryPort, &previousSerialConfig);
        serialRelease(mspTelemetryPort);

        //waitForSerialPortToFinishTransmitting(mspTelemetryPort); // FIXME locks up the system

        serialConfigure(mspTelemetryPort, &mspTelemetrySerialPortConfig);
        beginSerialPortFunction(mspTelemetryPort, FUNCTION_TELEMETRY);
    } else {
        mspTelemetryPort = openSerialPort(FUNCTION_TELEMETRY, &mspTelemetrySerialPortConfig);
    }
    mspSetTelemetryPort(mspTelemetryPort);
}

uint32_t getMSPTelemetryProviderBaudRate(void)
{
    return MSP_TELEMETRY_BAUDRATE;
}

#endif
