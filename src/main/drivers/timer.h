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
#include "resource.h"
#include "rcc.h"
#include "io.h"

#define TIM_N(i) (1 << (i))

#if defined(STM32F303)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#elif defined(STM32F10X)
typedef uint16_t timCCR_t;
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
typedef uint16_t timCNT_t;
#elif defined(UNIT_TEST)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#else
# error "Unknown CPU defined"
#endif

// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerCCHandlerRec_s timerCCHandlerRec_t;
typedef struct timerOvrHandlerRec_s timerOvrHandlerRec_t;
typedef void timerCCHandlerCallback(timerCCHandlerRec_t* self, uint16_t capture);
typedef void timerOvrHandlerCallback(timerOvrHandlerRec_t* self, uint16_t capture);

typedef struct timerRec_s timerRec_t;
typedef struct timerDef_s timerDef_t;
typedef struct ioDef_s ioDef_t;
typedef struct timerChRec_s timerChRec_t;
typedef struct timerChDef_s timerChDef_t;

// declared here, defined in target
extern const timerChDef_t timerQueueChDef;

// define available timers
#if defined(STM32F10X)
# include "drivers/iodef_timer_stm32f10x.h"
#elif defined(STM32F303xC)
# include "drivers/iodef_timer_stm32f30x.h"
#else
# warning "Unsupported target type"
#endif



void timerChConfigIC(timerChRec_t *timCh, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(timerChRec_t *timCh, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(timerChRec_t *timCh, bool polarityRising);

volatile timCCR_t* timerChCCR(timerChRec_t *timCh);
volatile timCCR_t* timerChCCRLo(timerChRec_t *timCh);
volatile timCCR_t* timerChCCRHi(timerChRec_t *timCh);
volatile timCNT_t* timerChCNT(timerChRec_t *timCh);
TIM_TypeDef* timerChTIM(timerChRec_t *timCh);

void timerChConfigOCPwm(timerChRec_t *timCh, uint16_t value);
void timerChConfigOC(timerChRec_t *timCh, bool outEnable, bool stateHigh);

void timerChConfigGPIO(timerChRec_t *timCh, ioConfig_t cfg);

void timerCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(timerChRec_t *timCh, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(timerChRec_t *timCh, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(timerChRec_t *timCh, FunctionalState newState);
void timerChITConfig(timerChRec_t *timCh, FunctionalState newState);
void timerChClearCCFlag(timerChRec_t *timCh);
void timerChIOWrite(timerChRec_t *timCh, bool value);

timerChRec_t* timerChInit(const timerChDef_t *def, resourceOwner_t owner, resourceType_t resources, int irqPriority, uint16_t period, int timerFrequency);
void timerChRelease(timerChRec_t* timCh);

resourceType_t timerChGetResources(timerChRec_t *timCh);
timerChRec_t* timerChRecDual(timerChRec_t *timCh);

void timerInit(void);
void timerStart(void);
void timerForceOverflow(timerRec_t *timRec);

TIM_TypeDef* timerChDef_TIM(const timerChDef_t* timChDef);
ioRec_t* timerChDef_IO(const timerChDef_t* timChDef);
resourceType_t timerChDef_GetResources(const timerChDef_t* timChDef);
timerChRec_t* timerChDef_TimChRec(const timerChDef_t* timChDef);
timerRec_t* timerChDef_TimRec(const timerChDef_t* timChDef);
