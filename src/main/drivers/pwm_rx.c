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
#include "build_config.h"

#include "common/utils.h"

#include "system.h"

#include "nvic.h"
#include "gpio.h"
#include "timer.h"

#include "pwm_mapping.h"

#include "pwm_rx.h"

#define PPM_CAPTURE_COUNT 12
#define PWM_INPUT_PORT_COUNT 8

#if PPM_CAPTURE_COUNT > MAX_PWM_INPUT_PORTS
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX ~0   // use maximum filter length

static inputFilteringMode_e inputFilteringMode;

typedef enum {
    INPUT_MODE_PPM,
    INPUT_MODE_PWM,
} pwmInputMode_t;

typedef struct {
    pwmInputMode_t mode;
    uint8_t channel; // only used for pwm, ignored by ppm

    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;

    uint8_t missedEvents;

    const timerChDef_t *timChDef;
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0x10000
#define PWM_TIMER_PERIOD 0x10000

static uint8_t ppmFrameCount = 0;
static uint8_t lastPPMFrameCount = 0;
static uint8_t ppmCountShift = 0;

typedef struct ppmDevice {
    uint8_t  pulseIndex;
    uint32_t previousTime;
    uint32_t currentTime;
    uint32_t deltaTime;
    uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
    uint32_t largeCounter;
    int8_t   numChannels;
    int8_t   numChannelsPrevFrame;
    uint8_t  stableFramesSeenCount;

    bool     tracking;
} ppmDevice_t;

ppmDevice_t ppmDev;


#define PPM_IN_MIN_SYNC_PULSE_US    2700    // microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US 750     // microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US 2250    // microseconds
#define PPM_STABLE_FRAMES_REQUIRED_COUNT    25
#define PPM_IN_MIN_NUM_CHANNELS     4
#define PPM_IN_MAX_NUM_CHANNELS     PWM_PORTS_OR_PPM_CAPTURE_COUNT


bool isPPMDataBeingReceived(void)
{
    return (ppmFrameCount != lastPPMFrameCount);
}

void resetPPMDataReceivedState(void)
{
    lastPPMFrameCount = ppmFrameCount;
}

#define MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID 4

void pwmRxInit(inputFilteringMode_e initialInputFilteringMode)
{
    inputFilteringMode = initialInputFilteringMode;
}

static void ppmInit(void)
{
    ppmDev.pulseIndex   = 0;
    ppmDev.previousTime = 0;
    ppmDev.currentTime  = 0;
    ppmDev.deltaTime    = 0;
    ppmDev.largeCounter = 0;
    ppmDev.numChannels  = -1;
    ppmDev.numChannelsPrevFrame = -1;
    ppmDev.stableFramesSeenCount = 0;
    ppmDev.tracking     = false;
}

static void ppmOverflowCallback(timerOvrHandlerRec_t* cbRec, uint16_t capture)
{
    UNUSED(cbRec);
    ppmDev.largeCounter += capture + 1;
}

static void ppmEdgeCallback(timerCCHandlerRec_t* cbRec, uint16_t capture)
{
    UNUSED(cbRec);
    int32_t i;

    /* Shift the last measurement out */
    ppmDev.previousTime = ppmDev.currentTime;

    /* Grab the new count */
    ppmDev.currentTime  = capture;

    /* Convert to 32-bit timer result */
    ppmDev.currentTime += ppmDev.largeCounter;

    // Divide by 8 if Oneshot125 is active and this is a CC3D board
    ppmDev.currentTime = ppmDev.currentTime >> ppmCountShift;

    /* Capture computation */
    ppmDev.deltaTime    = ppmDev.currentTime - ppmDev.previousTime;

    ppmDev.previousTime = ppmDev.currentTime;

#if 0
    static uint32_t deltaTimes[20];
    static uint8_t deltaIndex = 0;

    deltaIndex = (deltaIndex + 1) % 20;
    deltaTimes[deltaIndex] = ppmDev.deltaTime;
#endif

    /* Sync pulse detection */
    if (ppmDev.deltaTime > PPM_IN_MIN_SYNC_PULSE_US) {
        if (ppmDev.pulseIndex == ppmDev.numChannelsPrevFrame
            && ppmDev.pulseIndex >= PPM_IN_MIN_NUM_CHANNELS
            && ppmDev.pulseIndex <= PPM_IN_MAX_NUM_CHANNELS) {
            /* If we see n simultaneous frames of the same
               number of channels we save it as our frame size */
            if (ppmDev.stableFramesSeenCount < PPM_STABLE_FRAMES_REQUIRED_COUNT) {
                ppmDev.stableFramesSeenCount++;
            } else {
                ppmDev.numChannels = ppmDev.pulseIndex;
            }
        } else {
            ppmDev.stableFramesSeenCount = 0;
        }

        /* Check if the last frame was well formed */
        if (ppmDev.pulseIndex == ppmDev.numChannels && ppmDev.tracking) {
            /* The last frame was well formed */
            for (i = 0; i < ppmDev.numChannels; i++) {
                captures[i] = ppmDev.captures[i];
            }
            for (i = ppmDev.numChannels; i < PPM_IN_MAX_NUM_CHANNELS; i++) {
                captures[i] = PPM_RCVR_TIMEOUT;
            }
            ppmFrameCount++;
        }

        ppmDev.tracking   = true;
        ppmDev.numChannelsPrevFrame = ppmDev.pulseIndex;
        ppmDev.pulseIndex = 0;

        /* We rely on the supervisor to set captureValue to invalid
           if no valid frame is found otherwise we ride over it */
    } else if (ppmDev.tracking) {
        /* Valid pulse duration 0.75 to 2.5 ms*/
        if (ppmDev.deltaTime > PPM_IN_MIN_CHANNEL_PULSE_US
            && ppmDev.deltaTime < PPM_IN_MAX_CHANNEL_PULSE_US
            && ppmDev.pulseIndex < PPM_IN_MAX_NUM_CHANNELS) {
            ppmDev.captures[ppmDev.pulseIndex] = ppmDev.deltaTime;
            ppmDev.pulseIndex++;
        } else {
            /* Not a valid pulse duration */
            ppmDev.tracking = false;
            for (i = 0; i < PWM_PORTS_OR_PPM_CAPTURE_COUNT; i++) {
                ppmDev.captures[i] = PPM_RCVR_TIMEOUT;
            }
        }
    }
}

#define MAX_MISSED_PWM_EVENTS 10

bool isPWMDataBeingReceived(void)
{
    int channel;
    for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) {
        if (captures[channel] != PPM_RCVR_TIMEOUT) {
            return true;
        }
    }
    return false;
}

static void pwmOverflowCallback(timerOvrHandlerRec_t* cbRec, uint16_t capture)
{
    UNUSED(capture);
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, overflowCb);

    if (++pwmInputPort->missedEvents > MAX_MISSED_PWM_EVENTS) {
        captures[pwmInputPort->channel] = PPM_RCVR_TIMEOUT;
        pwmInputPort->missedEvents = 0;
    }
}

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, uint16_t capture)
{
    pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, edgeCb);
    const timerChDef_t *timChDef = pwmInputPort->timChDef;

    if (pwmInputPort->state == 0) {
        pwmInputPort->rise = capture;
        pwmInputPort->state = 1;
        timerChICPolarity(timChDef, false);
    } else {
        pwmInputPort->fall = capture;

        // compute and store capture
        pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
        captures[pwmInputPort->channel] = pwmInputPort->capture;

        // switch state
        pwmInputPort->state = 0;
        timerChICPolarity(timChDef, true);
        pwmInputPort->missedEvents = 0;
    }
}

void pwmInConfig(const timerChDef_t *timChDef, uint8_t channel)
{
    pwmInputPort_t *self = &pwmInputPorts[channel];

    self->state = 0;
    self->missedEvents = 0;
    self->channel = channel;
    self->mode = INPUT_MODE_PWM;
    self->timChDef = timChDef;

    timerChInit(timChDef, OWNER_PWMINPUT, RESOURCE_INPUT | RESOURCE_TIMER, NVIC_PRIO_TIMER, 0, PWM_TIMER_HZ);
    timerChConfigGPIO(timChDef, IOCFG_IPD);
    timerChConfigIC(timChDef, true, INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX);

    timerCCHandlerInit(&self->edgeCb, pwmEdgeCallback);
    timerOvrHandlerInit(&self->overflowCb, pwmOverflowCallback);
    timerChConfigCallbacks(timChDef, &self->edgeCb, &self->overflowCb);
}

#define FIRST_PWM_PORT 0

void ppmAvoidPWMTimerClash(const timerChDef_t *timChDef, TIM_TypeDef *sharedPwmTimer)
{
    if (timChDef->tim == sharedPwmTimer) {
        ppmCountShift = 3;  // Divide by 8 if the timer is running at 8 MHz
    }
}

void ppmInConfig(const timerChDef_t *timChDef)
{
    ppmInit();

    pwmInputPort_t *self = &pwmInputPorts[FIRST_PWM_PORT];

    self->mode = INPUT_MODE_PPM;
    self->timChDef = timChDef;

    timerChInit(timChDef, OWNER_PPMINPUT, RESOURCE_INPUT | RESOURCE_TIMER, NVIC_PRIO_TIMER, 0, PWM_TIMER_HZ);
    timerChConfigGPIO(timChDef, IOCFG_IPD);
    timerChConfigIC(timChDef, true, 0);

    timerCCHandlerInit(&self->edgeCb, ppmEdgeCallback);
    timerOvrHandlerInit(&self->overflowCb, ppmOverflowCallback);
    timerChConfigCallbacks(timChDef, &self->edgeCb, &self->overflowCb);
}

uint16_t ppmRead(uint8_t channel)
{
    return captures[channel];
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
