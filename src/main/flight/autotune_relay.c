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
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#ifdef AUTOTUNE

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/config.h"
#include "blackbox/blackbox.h"

// exceeding this angle will immediately abort autotune
// both axes are checked, not just tuned one
#define RELAY_ABORT_ANGLE 450

typedef enum {
    atStateIdle = 0,
    atStateRelayInit,
    atStateRelay,
    atStateRelayDone,
    atStateAbort,
} autotuneState_e;

static autotuneState_e state;        // state of autotune
static uint8_t axis;                 // axis we are currently working on
static uint32_t relayWatchdog;       // watchdog sor current state
static int autotuneRefHeading;       // 'zero' heading for autotune
// 0 - position relay / 1 - angular speed relay
//                                           ROLL       PITCH      YAW
static int16_t relayHysteresis[3][2]    = {{30, 100}, {30, 100}, {100, 100}};  // relay will flip after exceeding this threshold (hysteresis is twice this value)
static int16_t relayMagnitude[3][2]     = {{40, 200}, {40, 200}, {200,   0}};  // output value of relay
static int16_t relayMagnitudeHalf[3][2] = {{ 0,  70}, { 0,  70}, { 70,   0}};  // half-frequency parasitic relay magnitude
static int16_t relayCountTarget[3][2]   = {{-1,  50}, {-1,  50}, { 50,  -1}};  // number of direction reversals
static int16_t relayDirection[2] = {0, 0};      // sign of half-plane autotune value is currently in
static int relayCount[2];                       // number of zero crossings (count each separatelly)

bool isAutotuneIdle(void)
{
    return state == atStateIdle;
}

int16_t autotunePIDRelay(void)
{
    int16_t currentDelta[2] = {0, 0};

    switch(axis) {
    case FD_ROLL:
    case FD_PITCH:
        currentDelta[0] = inclination.raw[axis];
        break;
    case FD_YAW:
        currentDelta[0] = (heading - autotuneRefHeading) * 10;   // convert to decidegrees
        // normalize delta to -1799 .. 1800
        if(currentDelta[0] <= -1800)                             // 180 should be positive
            currentDelta[0] += 3600;
        else if(currentDelta[0] > 1800)
            currentDelta[0] -= 3600;
        break;
    }
    // angular speed
    currentDelta[1] = gyroADC[axis];

    if(ABS(inclination.raw[FD_ROLL]) > RELAY_ABORT_ANGLE
       || ABS(inclination.raw[FD_PITCH]) > RELAY_ABORT_ANGLE) {
        state = atStateAbort;
    }

#ifdef BLACKBOX
    flightLogEvent_autotuneCycleStart_t eventData;
    bool logEvent = false;
    eventData.phase = state;
    eventData.cycle = axis;
    eventData.p = ((relayCount[0] & 0x0f) << 4) | (relayCount[1] & 0x0f);
    eventData.i = 0;
    eventData.d = 0;
#endif

    switch(state) {
    case atStateRelayInit:
        for(int i = 0; i < 2; i++) {
            // determine current relay direction
            relayDirection[i] = (currentDelta[i] > 0) ? 1 : -1;
            relayCount[i] = 0;

            eventData.d |= (relayDirection[i] > 0) << i;
            eventData.d |= 1 << (i + 2);
        }
        relayWatchdog = millis() + 5000;
        state = atStateRelay;

        eventData.d |= 0x10;  // relay init
        logEvent = true;
        break;
    case atStateRelay: {
        if(cmp32(relayWatchdog, millis()) < 0) {
            state = atStateAbort;
            eventData.d |= 0x80;
            logEvent = true;
        }
        int val = 0;
        for(int i = 0; i < 2; i++) {
            if(relayDirection[i] * currentDelta[i] < -relayHysteresis[axis][i]) {
                // just crosed zero position, switch relay
                relayDirection[i] *= -1;
                relayCount[i]++;
                if(i == 0) {
                    // time to return back to level position, ignore speed crossing
                    relayWatchdog = millis() + 5000;
                }
                logEvent = true;
                eventData.d |= 1 << (i + 2);              // mark relay change
                if(relayCountTarget[axis][i] >= 0 && relayCount[i] >= relayCountTarget[axis][i]) {
                    state = atStateRelayDone;
                    eventData.d |= 0x40;
                }
            }
            eventData.d |= (relayDirection[i] > 0) << i;  // store relay direction into lowest two bits

            val += -relayDirection[i] * relayMagnitude[axis][i];
            val += ((relayCount[i] & 2) ? 1 : -1) * relayMagnitudeHalf[axis][i];
        }
        if(state == atStateRelay) {  // no error
            eventData.i = val / 10;
            axisPID[axis] = val;
        }
        break;
    }
    default: ;
    }
#ifdef BLACKBOX
    if(logEvent)
        blackboxLogEvent(FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START, (flightLogEventData_t*)&eventData);
#endif
    return 0;
}

void autotuneReset(void)
{
    state = atStateIdle;
}

// called when entering autotune state
void autotuneActivated(void)
{
    switch(state) {
    case atStateIdle:
        axis = 0;
        break;
    case atStateAbort:
    case atStateRelayDone:
        axis = (axis + 1) % 3;  // TODO
        break;
    default:
        // we should not be here !
        return;
    }

    autotuneRefHeading = heading;    // remember heading when starting starting autotune phase
    state = atStateRelayInit;
    relayDirection[0] = 0;
    relayDirection[1] = 0;
}

void autotuneDeactivated(void)
{
    flightLogEvent_autotuneCycleStart_t eventData;
    switch(state) {
    case atStateRelayInit:
    case atStateRelay:
        eventData.phase = state;
        eventData.cycle = axis;
        eventData.p = 0;
        eventData.i = 0;
        eventData.d = 0x20;
        eventData.rising = 0;
        blackboxLogEvent(FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START, (flightLogEventData_t*)&eventData);

        state = atStateAbort;
    default:;
    }
}

// user want's do save results, is that neccessary/desirable
bool autotuneShouldSavePIDs(void)
{
    return false;
}

#endif
