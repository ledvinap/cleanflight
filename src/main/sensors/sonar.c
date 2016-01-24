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
#include <math.h>

#include "platform.h"
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu
float baro_cf_vel;                      // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
float baro_cf_alt;                      // apply CF to use ACC for height estimation

#ifdef SONAR
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
float sonarMaxTiltCos;

static int32_t calculatedAltitude;

const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig)
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
#if 1
    static const sonarHardware_t sonarPWM56 = {
        .triggerIOTag = IO_TAG(PB8),   // PWM5 (PB8) - 5v tolerant
        .echoIOTag = IO_TAG(PB9)       // PWM6 (PB9) - 5v tolerant
    };
    static const sonarHardware_t sonarRC78 = {
        .triggerIOTag = IO_TAG(PB0),   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echoIOTag = IO_TAG(PB1),      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
    };
    // If we are using softserial, parallel PWM or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_SOFTSERIAL)
            || feature(FEATURE_RX_PARALLEL_PWM )
            || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC)) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
# else
    static const sonarHardware_t const sonarHardware = {
        .triggerIOTag = IO_TAG(PB7),
        .echoIOTag = IO_TAG(PB6),
    };
    return &sonarHardware;
# endif
#elif defined(OLIMEXINO)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .triggerIOTag = IO_TAG(PB0),    // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echoIOTag = IO_TAG(PB1),       // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
    };
    return sonarHardware;
#elif defined(SPRACINGF3)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .triggerIOTag = IO_TAG(PB0),    // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echoIOTag = IO_TAG(PB1),       // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
    };
    return &sonarHardware;
#elif defined(SPARKY)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .triggerIOTag = IO_TAG(PA2),    // PWM6 (PA2) - only 3.3v ( add a 1K Ohms resistor )
        .echoIOTag = IO_TAG(PB1),       // PWM7 (PB1) - only 3.3v ( add a 1K Ohms resistor )
    };
    return &sonarHardware;
#elif defined(UNIT_TEST)
    UNUSED(batteryConfig);
    return 0;
#else
# error Sonar not defined for target
#endif
}

void sonarInit(const sonarHardware_t *sonarHardware)
{
    sonarRange_t sonarRange;

    hcsr04_Init(sonarHardware, &sonarRange);
    sensorsSet(SENSOR_SONAR);
    sonarMaxRangeCm = sonarRange.maxRangeCm;
    sonarCfAltCm = sonarMaxRangeCm / 2;
    sonarMaxTiltDeciDegrees =  sonarRange.detectionConeExtendedDeciDegrees / 2;
    sonarMaxTiltCos = cos_approx(sonarMaxTiltDeciDegrees / 10.0f * RAD);
    sonarMaxAltWithTiltCm = sonarMaxRangeCm * sonarMaxTiltCos;
    calculatedAltitude = SONAR_OUT_OF_RANGE;
}

#define DISTANCE_SAMPLES_MEDIAN 5

static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    if (newSonarReading > SONAR_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
    else
        return newSonarReading;
}

void sonarUpdate(void)
{
    hcsr04_Poll();
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarRead(void)
{
    int32_t distance = hcsr04_GetDistance();
    if (distance > HCSR04_MAX_RANGE_CM)
        distance = SONAR_OUT_OF_RANGE;

    return applySonarMedianFilter(distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarCalculateAltitude(int32_t sonarDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the sonar cone
    if (cosTiltAngle <= sonarMaxTiltCos)
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
        calculatedAltitude = sonarDistance * cosTiltAngle;
    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or SONAR_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonarGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
