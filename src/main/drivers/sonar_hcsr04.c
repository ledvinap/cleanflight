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

void hcsr04_extiHandler(extiCallbackRec_t* cb)
{
    static uint32_t timing_start;
    uint32_t timing_stop;

    UNUSED(cb);

    if (IO_DigitalRead(sonarHardware->echoIO) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        measurement = cmp32(timing_stop, timing_start);
    }
}

void hcsr04_Init(const sonarHardware_t *initialSonarHardware)
{
    sonarHardware = initialSonarHardware;

    if(sonarHardware->triggerIO != sonarHardware->echoIO) {
        // separate trigger pin, configure it as output
        IO_ConfigGPIO(sonarHardware->triggerIO, Mode_Out_PP);
    }
    // ep - echo pin, configure as input (even if same as trigger)
    IO_ConfigGPIO(sonarHardware->echoIO, Mode_IN_FLOATING);

    // setup external interrupt on echo pin
    EXTIHandlerInit(&hcsr04_extiCallbackRec, hcsr04_extiHandler);
    EXTIConfig(sonarHardware->echoIO, &hcsr04_extiCallbackRec, NVIC_PRIO_SONAR_EXTI, EXTI_Trigger_Rising_Falling); // TODO - priority!
    EXTIEnable(sonarHardware->echoIO, true);

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
    // TODO - this needs some analysis to avoid race conditions
    if(sonarHardware->triggerIO != sonarHardware->echoIO) {
        IO_DigitalWrite(sonarHardware->triggerIO, true);
        //  The width of trig signal must be greater than 10us
        delayMicroseconds(10);
        IO_DigitalWrite(sonarHardware->triggerIO, false);
    } else {
        EXTIEnable(sonarHardware->echoIO, false);
        IO_ConfigGPIO(sonarHardware->echoIO, Mode_Out_PP);
        IO_DigitalWrite(sonarHardware->echoIO, true);
        delayMicroseconds(10);
        IO_DigitalWrite(sonarHardware->echoIO, false);
        IO_ConfigGPIO(sonarHardware->echoIO, Mode_IN_FLOATING);
        // TODO - there may be race if we don't enable EXTI soon enough
    }
    EXTIEnable(sonarHardware->echoIO, true);
}

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
