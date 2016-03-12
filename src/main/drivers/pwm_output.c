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

#include <platform.h>

#include "gpio.h"
#include "nvic.h"
#include "timer.h"

#include "flight/failsafe.h" // FIXME dependency into the main code from a driver

#include "pwm_mapping.h"

#include "pwm_output.h"

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

typedef struct {
    volatile timCCR_t *ccr;
    timerChRec_t *timChRec;
    uint16_t period;
    pwmWriteFuncPtr pwmWritePtr;
} pwmOutputPort_t;

static pwmOutputPort_t pwmOutputPorts[MAX_PWM_OUTPUT_PORTS];

static pwmOutputPort_t *motors[MAX_PWM_MOTORS];

#ifdef USE_SERVOS
static pwmOutputPort_t *servos[MAX_PWM_SERVOS];
#endif

static uint8_t allocatedOutputPortCount = 0;

static pwmOutputPort_t *pwmOutConfig(const timerChDef_t *timChDef, resourceOwner_t owner, int hz, uint16_t period, uint16_t value)
{
    pwmOutputPort_t *p = &pwmOutputPorts[allocatedOutputPortCount++];

    timerChRec_t *timChRec = timerChInit(timChDef, owner, RESOURCE_OUTPUT | RESOURCE_TIMER, NVIC_PRIO_TIMER_PWMOUT, period, hz);
    timerChConfigGPIO(timChRec, IOCFG_AF_PP);
    timerChConfigOCPwm(timChRec, value);

    p->ccr = timerChCCR(timChRec);
    p->period = period;
    p->timChRec = timChRec;

    return p;
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value - 1000) * motors[index]->period / 1000;
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = value;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (motors[index] && index < MAX_MOTORS)
        motors[index]->pwmWritePtr(index, value);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    uint8_t index;

    for(index = 0; index < motorCount; index++){
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        *motors[index]->ccr = 0;
    }
}

// TODO - timer internals should be opaque
#include "timer_impl.h"

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    uint8_t index;
    timerRec_t *lastTimerPtr = NULL;

    for(index = 0; index < motorCount; index++) {
        // Force the timer to overflow if it's the first motor to output, or if we change timers
        if(motors[index]->timChRec->timRec != lastTimerPtr) {
            lastTimerPtr = motors[index]->timChRec->timRec;

            timerForceOverflow(lastTimerPtr);
        }

        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index]->ccr = 0;
    }
}

bool isMotorBrushed(uint16_t motorPwmRate)
{
    return (motorPwmRate > 500);
}

void pwmBrushedMotorConfig(const timerChDef_t *timChDef, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse)
{
    motors[motorIndex] = pwmOutConfig(timChDef, OWNER_PWMOUTPUT_MOTOR, PWM_BRUSHED_TIMER_HZ, PWM_BRUSHED_TIMER_HZ / motorPwmRate, idlePulse);
    motors[motorIndex]->pwmWritePtr = pwmWriteBrushed;
}

void pwmBrushlessMotorConfig(const timerChDef_t *timChDef, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse)
{
    motors[motorIndex] = pwmOutConfig(timChDef, OWNER_PWMOUTPUT_FAST, PWM_TIMER_HZ, PWM_TIMER_HZ / motorPwmRate, idlePulse);
    motors[motorIndex]->pwmWritePtr = pwmWriteStandard;
}

void pwmOneshotMotorConfig(const timerChDef_t *timChDef, uint8_t motorIndex)
{
    motors[motorIndex] = pwmOutConfig(timChDef, OWNER_PWMOUTPUT_ONESHOT, ONESHOT125_TIMER_HZ, (uint16_t)0x10000, 0);
    motors[motorIndex]->pwmWritePtr = pwmWriteStandard;
}

#ifdef USE_SERVOS
void pwmServoConfig(const timerChDef_t *timChDef, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse)
{
    servos[servoIndex] = pwmOutConfig(timChDef, OWNER_PWMOUTPUT_SERVO, PWM_TIMER_HZ, PWM_TIMER_HZ / servoPwmRate, servoCenterPulse);
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (servos[index] && index < MAX_SERVOS)
        *servos[index]->ccr = value;
}
#endif
