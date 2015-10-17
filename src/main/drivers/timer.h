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

#include "resource.h"
#include "rcc.h"
#include "io.h"

#if defined(STM32F303xC)
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

// overflow and capture handlers. Caller is responsible for memory allocation
// different types used for capture and overflow - multiple overflow handlers are implemented as linked list
typedef struct timerCCHandlerRec_s timerCCHandlerRec_t;
typedef struct timerOvrHandlerRec_s timerOvrHandlerRec_t;
typedef void timerCCHandlerCallback(timerCCHandlerRec_t *self, uint16_t capture);
typedef void timerOvrHandlerCallback(timerOvrHandlerRec_t *self, uint16_t capture);
typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;
typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

// structures used to precify timer and timerChannel
// only pointer should be used in general code (so opaque decolaration is ok)
// include "timer_impl.h" if you really need it's internals
typedef struct timerRec_s timerRec_t;
typedef struct timerChRec_s timerChRec_t;
// timer channel definition
typedef struct timerChDef_s timerChDef_t;


// TIMx access functions
volatile timCCR_t* timerChCCR(timerChRec_t *timCh);
volatile timCCR_t* timerChCCRLo(timerChRec_t *timCh);
volatile timCCR_t* timerChCCRHi(timerChRec_t *timCh);
volatile timCNT_t* timerChCNT(timerChRec_t *timCh);
TIM_TypeDef* timerChTIM(timerChRec_t *timCh);


// configure channel in input mode
void timerChConfigIC(timerChRec_t *timCh, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(timerChRec_t *timCh, bool polarityRising, unsigned inputFilterSamples);
// fast function to set input polarity
void timerChICPolarity(timerChRec_t *timCh, bool polarityRising);

// configure channel in output compare mode
void timerChConfigOCPwm(timerChRec_t *timCh, uint16_t value);
void timerChConfigOC(timerChRec_t *timCh, bool outEnable, bool stateHigh);

// set underlying GPIO configuration
void timerChConfigGPIO(timerChRec_t *timCh, ioConfig_t cfg);

// channel interrupt handling
void timerCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(timerChRec_t *timCh, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(timerChRec_t *timCh, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(timerChRec_t *timCh, FunctionalState newState);
void timerChITConfig(timerChRec_t *timCh, FunctionalState newState);
void timerChClearCCFlag(timerChRec_t *timCh);
void timerChForceOverflow(timerChRec_t *timCh);
// write to IO
void timerChIOWrite(timerChRec_t *timCh, bool value);
// get resources used by IO
resourceType_t timerChGetResources(timerChRec_t *timCh);
// return second timerCh from pair (1-2, 3-4)
timerChRec_t* timerChRecDual(timerChRec_t *timCh);

// channel initialization. Allocates specified channel and does basic configuration
// timer is sconfigured started in this function if necessary
timerChRec_t* timerChInit(const timerChDef_t *def, resourceOwner_t owner, resourceType_t resources, int irqPriority, uint16_t period, int timerFrequency);
// release channel. TODO - unimplemented
void timerChRelease(timerChRec_t* timCh);

// init timer subsystem
void timerInit(void);
// start timers after initialization. Not implemented now, timers are started immediately when channel is configured
void timerStart(void);

void timerForceOverflow(timerRec_t *timRec);

// functions to query timerChDef, used in CF initialization phase to resolve conflicts
TIM_TypeDef* timerChDef_TIM(const timerChDef_t* timChDef);
IO_t timerChDef_IO(const timerChDef_t* timChDef);
resourceType_t timerChDef_GetResources(const timerChDef_t* timChDef);
timerChRec_t* timerChDef_TimChRec(const timerChDef_t* timChDef);
timerRec_t* timerChDef_TimRec(const timerChDef_t* timChDef);
