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
#include "system.h"
#include "gpio.h"
#include "common/utils.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"

#include "sonar_hcsr04.h"

#ifdef SONAR

#define SONAR_INTERVAL 60    // time between normal measurements

#ifndef USE_EXTI
# error "HCSR-04 driver needs EXTI driver"
#endif

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static uint32_t lastMeasurementAt;
static volatile int32_t measurement = -1;
static sonarHardware_t const *sonarHardware;

extiCallbackRec_t hcsr04_extiCallbackRec;
static IO_t echoIO, triggerIO;

void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
    static uint32_t timing_start;
    uint32_t timing_stop;

    UNUSED(cb);

    if (IORead(echoIO)) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        measurement = cmp32(timing_stop, timing_start);
        // disable exti until we request another pulse
        EXTIEnable(echoIO, false);
    }
}

void hcsr04_Init(const sonarHardware_t *initialSonarHardware)
{
    sonarHardware = initialSonarHardware;
    // both pins must be defined, but may be the same
    if(!sonarHardware->triggerIO || !sonarHardware->echoIO)
        return;

    if(sonarHardware->triggerIO == sonarHardware->echoIO) {
        // single-wire configuration
        triggerIO = echoIO = IOGetByTag(sonarHardware->triggerIO);
        IOInit(triggerIO, OWNER_SONAR, RESOURCE_IO | RESOURCE_EXTI);
        // start in input mode
        IOConfigGPIO(echoIO, IOCFG_IN_FLOATING);
    } else {
        triggerIO = IOGetByTag(sonarHardware->triggerIO);
        IOInit(triggerIO, OWNER_SONAR, RESOURCE_OUTPUT);
        IOConfigGPIO(triggerIO, IOCFG_OUT_PP);
        echoIO = IOGetByTag(sonarHardware->echoIO);
        IOInit(echoIO, OWNER_SONAR, RESOURCE_INPUT | RESOURCE_EXTI);
        IOConfigGPIO(echoIO, IOCFG_IN_FLOATING);
    }
    // setup external interrupt on echo pin
    EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
    EXTIConfig(echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, EXTI_Trigger_Rising_Falling); // TODO - priority!
    EXTIEnable(echoIO, true);

    lastMeasurementAt = 0;      // force 1st measurement in hcsr04_get_distance()
}

// measurement reading is done asynchronously, using interrupt
void hcsr04_Poll(void)
{
    uint32_t now = millis();

    if (cmp32(now, lastMeasurementAt) < SONAR_INTERVAL) {
        // the repeat interval of trig signal should be greater than 60ms
        // to avoid interference between connective measurements.
        return;
    }

    lastMeasurementAt = now;

    if(triggerIO != echoIO) {
        IOHi(triggerIO);
        //  The width of trig signal must be greater than 10us
        delayMicroseconds(10);
        IOLo(triggerIO);
    } else {
        EXTIEnable(echoIO, false);
        IOConfigGPIO(echoIO, IOCFG_OUT_PP);
        IOHi(echoIO);
        delayMicroseconds(10);
        IOLo(echoIO);
        IOConfigGPIO(echoIO, IOCFG_IN_FLOATING);
        // EXTI must be enabled before rising edge of measurement pulse
        // minimum delay is 8 40kHz pulses = 200us. Only single measurement will be lost if we don't make it
    }
    EXTIEnable(echoIO, true);
}

/**
 * Get the distance that was measured by the last pulse, in centimeters. When the ground is too far away to be
 * reliably read by the sonar, -1 is returned instead.
 */
int32_t hcsr04_GetDistance(void)
{
    // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance traveled.
    //
    // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
    int32_t distance = measurement / 59;

    // this sonar range is up to 4meter , but 3meter is the safe working range (+tilted and roll)
    if (distance > 300)
        distance = -1;

    return distance;
}
#endif
