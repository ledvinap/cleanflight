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
#include <common/axis.h>
#include "common/color.h"

#include "config/config.h"

//#include "drivers/light_ws2811strip.h"
//#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "flight/pid.h"
//#include "flight/failsafe.h"
#include "flight/mixer.h"
//#include "flight/navigation.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

//#include "config/runtime_config.h"
#include "rx/rx.h"
//#include "io/rc_controls.h"
#include "io/escservo.h"
#include "io/gps.h"
//#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"

//#include "config/runtime_config.h"
//#include "config/config.h"
#include "config/config_profile.h"

#include "telemetry/telemetry.h"
//#include "drivers/accgyro.h"



// System-wide
typedef struct master_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t mixerMode;
    uint32_t enabledFeatures;
    uint16_t loopTime;                       // imu loop time in us
    uint8_t loopTicks;                       // imu loop time in gyro periods
    uint8_t emf_avoidance;                   // change pll settings to avoid noise in the uhf band
    uint8_t i2c_highspeed;                  // Overclock i2c Bus for faster IMU readings
    uint8_t gyroSync;                       // Enable interrupt based loop
    uint8_t gyroSyncDenominator;            // Gyro sync Denominator

    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
#ifdef USE_SERVOS
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif
    // motor/esc/servo related stuff
    escAndServoConfig_t escAndServoConfig;
    flight3DConfig_t flight3DConfig;

    uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
    uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)

    // global sensor-related stuff

    sensorAlignmentConfig_t sensorAlignmentConfig;
    boardAlignment_t boardAlignment;

    int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device

    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t gyro_lpf;                       // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    float soft_gyro_lpf_hz;                 // Software based gyro filter in hz

    gyroConfig_t gyroConfig;

    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use

    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;

    batteryConfig_t batteryConfig;

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.

    failsafeConfig_t failsafeConfig;

    uint8_t retarded_arm;                   // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
    uint8_t small_angle;

    // mixer-related configuration
    mixerConfig_t mixerConfig;

    airplaneConfig_t airplaneConfig;

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

    serialConfig_t serialConfig;

#ifdef TELEMETRY
    telemetryConfig_t telemetryConfig;
#endif

#ifdef LED_STRIP
    ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH];
    hsvColor_t colors[CONFIGURABLE_COLOR_COUNT];
#endif

#ifdef TRANSPONDER
    uint8_t transponderData[6];
#endif

    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];

#ifdef BLACKBOX
    uint8_t blackbox_rate_num;
    uint8_t blackbox_rate_denom;
    uint8_t blackbox_device;
#endif

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
} master_t;

extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;
