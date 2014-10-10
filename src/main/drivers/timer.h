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

#include "gpio.h"

#include "callback.h"

#ifdef CHEBUZZF3
#define USABLE_TIMER_CHANNEL_COUNT 18
#endif

#ifdef CC3D
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#if !defined(USABLE_TIMER_CHANNEL_COUNT)
#define USABLE_TIMER_CHANNEL_COUNT 14
#endif

#if defined(STM32F303xC)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
#elif defined(STM32F10X)
typedef uint16_t timCCR_t;
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
#else 
# error "Unknown CPU defined"
#endif

typedef void timerCCCallback(void* data, uint16_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
} timerHardware_t;

extern const timerHardware_t timerHardware[];





void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallback *edgeCallback, timerCCCallback *overflowCallback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, void* data, timerCCCallback *edgeCallback);
void configureTimerChannelCallbacks(TIM_TypeDef *tim, uint8_t channel, void* data, timerCCCallback *edgeCallback, timerCCCallback *overflowCallback);

void timerChConfigIC(const timerHardware_t *channel, bool polarityRising);
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising);
void timerChICPolarity(const timerHardware_t *channel, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool outInvert);
void timerChCfgGPIO(const timerHardware_t* timHw, GPIO_Mode mode);
void timerChCfgCallbacks(const timerHardware_t *channel, void* data, timerCCCallback *edgeCallback, timerCCCallback *overflowCallback);
void timerChCfgCallbacksDual(const timerHardware_t *channel, void* data, timerCCCallback *edgeCallback1, timerCCCallback *edgeCallback2, timerCCCallback *overflowCallback);
void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState);
void timerConfigHandled(const timerHardware_t *timHw); // TODO?



