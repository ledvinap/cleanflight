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

#ifdef TELEMETRY

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/msp.h"
#include "telemetry/sport.h"

static bool isTelemetryConfigurationValid = false; // flag used to avoid repeated configuration checks
static bool telemetryEnabled = false;
static bool telemetryPortIsShared;

static telemetryConfig_t *telemetryConfig;

void useTelemetryConfig(telemetryConfig_t *telemetryConfigToUse)
{
    telemetryConfig = telemetryConfigToUse;
}

bool canUseTelemetryWithCurrentConfiguration(void)
{
    if (!feature(FEATURE_TELEMETRY)) {
        return false;
    }

    if (!canOpenSerialPort(FUNCTION_TELEMETRY)) {
        return false;
    }

    return true;
}

void initTelemetry()
{
    telemetryPortIsShared = isSerialPortFunctionShared(FUNCTION_TELEMETRY, FUNCTION_MSP);
    isTelemetryConfigurationValid = canUseTelemetryWithCurrentConfiguration();

    switch(telemetryConfig->telemetry_provider) {
    case TELEMETRY_PROVIDER_FRSKY:
        initFrSkyTelemetry(telemetryConfig);
        break;
    case TELEMETRY_PROVIDER_HOTT:
        initHoTTTelemetry(telemetryConfig);
        break;
    case TELEMETRY_PROVIDER_MSP:
        initMSPTelemetry(telemetryConfig);
        break;
    case TELEMETRY_PROVIDER_SPORT:
        initSPortTelemetry(telemetryConfig);
        break;
    }
    checkTelemetryState();
}

bool determineNewTelemetryEnabledState(void)
{
    bool enabled = true;

    if (telemetryPortIsShared) {
        if (telemetryConfig->telemetry_switch)
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool shouldChangeTelemetryStateNow(bool newState)
{
    return newState != telemetryEnabled;
}

uint32_t getTelemetryProviderBaudRate(void)
{
    switch(telemetryConfig->telemetry_provider) {
    case TELEMETRY_PROVIDER_FRSKY:
        return getFrSkyTelemetryProviderBaudRate();
    case TELEMETRY_PROVIDER_HOTT:
        return getHoTTTelemetryProviderBaudRate();
    case TELEMETRY_PROVIDER_MSP:
        return getMSPTelemetryProviderBaudRate();
    case TELEMETRY_PROVIDER_SPORT:
        return getSPortTelemetryProviderBaudRate();
    }
    return 0;
}

static void configureTelemetryPort(void)
{
    switch(telemetryConfig->telemetry_provider) {
    case TELEMETRY_PROVIDER_FRSKY:
        configureFrSkyTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_HOTT:
        configureHoTTTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_MSP:
        configureMSPTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_SPORT:
        configureSPortTelemetryPort();
        break;
    }
}


void freeTelemetryPort(void)
{
    switch(telemetryConfig->telemetry_provider) {
    case TELEMETRY_PROVIDER_FRSKY:
        freeFrSkyTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_HOTT:
        freeHoTTTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_MSP:
        freeMSPTelemetryPort();
        break;
    case TELEMETRY_PROVIDER_SPORT:
        freeSPortTelemetryPort();
        break;
    }
}

void checkTelemetryState(void)
{
    if (!isTelemetryConfigurationValid) {
        return;
    }

    bool newEnabledState = determineNewTelemetryEnabledState();

    if (!shouldChangeTelemetryStateNow(newEnabledState)) {
        return;
    }

    if (newEnabledState)
        configureTelemetryPort();
    else
        freeTelemetryPort();

    telemetryEnabled = newEnabledState;
}

void handleTelemetry(void)
{
    if (!isTelemetryConfigurationValid || !determineNewTelemetryEnabledState())
        return;

    if (!telemetryEnabled) {
        return;
    }

    switch(telemetryConfig->telemetry_provider) {
    case TELEMETRY_PROVIDER_FRSKY:
       handleFrSkyTelemetry();
        break;
    case TELEMETRY_PROVIDER_HOTT:
        handleHoTTTelemetry();
        break;
    case TELEMETRY_PROVIDER_MSP:
        handleMSPTelemetry();
        break;
    case TELEMETRY_PROVIDER_SPORT:
        handleSPortTelemetry();
        break;
    }
}
#endif
