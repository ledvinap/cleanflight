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
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"
#include "timer.h"
#include "timer_impl.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "config/config_master.h"

#include "drivers/serial_softserial.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

/*
    Configuration maps

    Note: this documentation is only valid for STM32F10x, for STM32F30x please read the code itself.

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

enum {
    MAP_TO_PPM_INPUT = 1,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
};

#if defined(NAZE) || defined(OLIMEXINO) || defined(NAZE32PRO) || defined(STM32F3DISCOVERY) || defined(EUSTM32F103RC) || defined(PORT103R)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     // Swap to servo if needed
#if 0
    // remapped uart and sonar is here ... 
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
#endif
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #5
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #8
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};
#endif

#ifdef CC3D
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #3
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};
#endif

#ifdef CJMCU
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
        0xFFFF
};

static const uint16_t airPWM[] = {
        0xFFFF
};
#endif

#ifdef COLIBRI_RACE
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    // TODO
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

static const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};
#endif

#if defined(SPARKY) || defined(ALIENWIIF3)
static const uint16_t multiPPM[] = {
    PWM11 | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM15
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM1
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM17
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), // TIM3
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8), // TIM2
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

static const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};
#endif

#ifdef CRAZYFLIE
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),

    PWM7  | (MAP_TO_PPM_INPUT << 8), // PPM input, avoid TIM3
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),

    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),

    0xFFFF
};

static const uint16_t airPPM[] = {
        0xFFFF
};

static const uint16_t airPWM[] = {
        0xFFFF
};

#endif

#ifdef SPRACINGF3
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #10
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), // server #6
    0xFFFF
};
#endif

#if defined(MOTOLAB)
static const uint16_t multiPPM[] = {
    PWM9  | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

static const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};
#endif

static const uint16_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static pwmIOConfiguration_t pwmIOConfiguration;

pwmIOConfiguration_t *pwmGetOutputConfiguration(void){
    return &pwmIOConfiguration;
}

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint16_t *setup;

    int channelIndex = 0;


    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM || init->useSerialRx)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < timerChannelMap_Count() && setup[i] != 0xFFFF; i++) {
        uint8_t timerIndex = setup[i] & 0x00FF;
        uint8_t type = (setup[i] & 0xFF00) >> 8;

        const timerChDef_t *timChDef = &timerChannelMap[timerIndex];

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2)
            continue;
#endif

#ifdef STM32F10X
        // skip UART2 ports
        if (init->useUART2 && (timerIndex == PWM3 || timerIndex == PWM4))
            continue;
#endif

#if defined(STM32F303xC) && defined(USE_USART3) && 0 // TODO!
        // skip UART3 ports (PB10/PB11)
        if (init->useUART3 && timerHardwarePtr->gpio == UART3_GPIO && (timerHardwarePtr->pin == UART3_TX_PIN || timerHardwarePtr->pin == UART3_RX_PIN))
            continue;
#endif



#ifdef USE_SOFTSERIAL1
        if (init->useSoftSerial
            && ( masterConfig.serialConfig.softserial_pins[SOFTSERIAL1][0] == timerIndex
                 ||  masterConfig.serialConfig.softserial_pins[SOFTSERIAL1][1] == timerIndex
                ) )
            continue;
#endif

#ifdef USE_SOFTSERIAL2
        if (init->useSoftSerial
            && (masterConfig.serialConfig.softserial_pins[SOFTSERIAL2][0] == timerIndex
                ||  masterConfig.serialConfig.softserial_pins[SOFTSERIAL2][1] == timerIndex
                ) )
            continue;
#endif

#ifdef LED_STRIP_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerChDef_TIM(timChDef) == LED_STRIP_TIMER)
                continue;
#if defined(STM32F303xC) && defined(WS2811_GPIO) && defined(WS2811_PIN)
            if (timerChDef_IO(timChDef) == DEFIO_IO(WS2811_IO))
                continue;
#endif
        }

#endif

#ifdef VBAT_ADC_IO
        if (init->useVbat && timerChDef_IO(timChDef) == DEFIO_IO(VBAT_ADC_IO))
            continue;
#endif

#ifdef RSSI_ADC_IO
        if (init->useRSSIADC && timerChDef_IO(timChDef) == DEFIO_IO(RSSI_ADC_IO))
            continue;
#endif

#ifdef CURRENT_METER_ADC_IO
        if (init->useCurrentMeterADC && timerChDef_IO(timChDef) == DEFIO_IO(CURRENT_METER_ADC_IO))
            continue;
#endif

        // TODO - SONAR
        if(timerChDef_GetResources(timChDef) & (RESOURCE_IO | RESOURCE_TIMER))
            continue;

        // hacks to allow current functionality
        if (type == MAP_TO_PWM_INPUT && !init->useParallelPWM)
            continue;

        if (type == MAP_TO_PPM_INPUT && !init->usePPM)
            continue;

#ifdef USE_SERVOS
        if (init->useServos && !init->airplane) {
#if defined(NAZE)
            // remap PWM9+10 as servos
            if ((timerIndex == PWM9 || timerIndex == PWM10) && timerChDef_TIM(timChDef) == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(COLIBRI_RACE)
            // remap PWM1+2 as servos
            if ((timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9) && timerChDef_TIM(timChDef) == TIM2)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(CC3D)
            // remap 10 as servo
            if (timerIndex == PWM10 && timerHardwarePtr->tim == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPARKY)
            // remap PWM1+2 as servos
            if ((timerIndex == PWM1 || timerIndex == PWM2) && timerChDef_TIM(timChDef) == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPRACINGF3)
            // remap PWM15+16 as servos
            if ((timerIndex == PWM15 || timerIndex == PWM16) && timerChDef_TIM(timChDef) == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(NAZE32PRO) || (defined(STM32F3DISCOVERY) && !defined(CHEBUZZF3))
            // remap PWM 5+6 or 9+10 as servos - softserial pin pairs require timer ports that use the same timer
            // TODO!
            if (init->useSoftSerial) {
                if (timerIndex == PWM5 || timerIndex == PWM6)
                    type = MAP_TO_SERVO_OUTPUT;
            } else {
                if (timerIndex == PWM9 || timerIndex == PWM10)
                    type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(MOTOLAB)
            // remap PWM 7+8 as servos
            if (timerIndex == PWM7 || timerIndex == PWM8)
                type = MAP_TO_SERVO_OUTPUT;
#endif
        }

        if (init->useChannelForwarding && !init->airplane) {
#if defined(NAZE) && defined(LED_STRIP_TIMER)
            // if LED strip is active, PWM5-8 are unavailable, so map AUX1+AUX2 to PWM13+PWM14
            if (init->useLEDStrip) {
                if (timerIndex >= PWM13 && timerIndex <= PWM14) {
                  type = MAP_TO_SERVO_OUTPUT;
                }
            } else
#endif
                // remap PWM5..8 as servos when used in extended servo mode
                if (timerIndex >= PWM5 && timerIndex <= PWM8)
                    type = MAP_TO_SERVO_OUTPUT;
        }
#endif

#ifdef CC3D
        if (init->useParallelPWM) {
            // Skip PWM inputs that conflict with timers used outputs.
            if ((type == MAP_TO_SERVO_OUTPUT || type == MAP_TO_MOTOR_OUTPUT) && (timChDef->tim == TIM2 || timChDef->tim == TIM3)) {
                continue;
            }
            if (type == MAP_TO_PWM_INPUT && timChDef->tim == TIM4) {
                continue;
            }

        }
#endif

        if (type == MAP_TO_PPM_INPUT) {
#ifdef CC3D
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timChDef, TIM4);
            }
#endif
#ifdef SPARKY
            if (init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timChDef, TIM2);
            }
#endif
            ppmInConfig(timChDef);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;
        } else if (type == MAP_TO_PWM_INPUT) {
            pwmInConfig(timChDef, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;
        } else if (type == MAP_TO_MOTOR_OUTPUT) {

            if (init->useOneshot) {
                pwmOneshotMotorConfig(timChDef, pwmOutputConfiguration.motorCount);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_ONESHOT|PWM_PF_OUTPUT_PROTOCOL_PWM;
            } else if (isMotorBrushed(init->motorPwmRate)) {
                pwmBrushedMotorConfig(timChDef, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;
            } else {
                pwmBrushlessMotorConfig(timChDef, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            }

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware =  timerChDef_TimChRec(timChDef);

            pwmIOConfiguration.motorCount++;

        } else if (type == MAP_TO_SERVO_OUTPUT) {
#ifdef USE_SERVOS
            pwmServoConfig(timChDef, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerChDef_TimChRec(timChDef);

            pwmIOConfiguration.servoCount++;
#endif
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}
