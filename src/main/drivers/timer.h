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

#define TIM_N(i) (1 << (i))

#include "target_timer.h"

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

struct timerRec_s;
struct timerDef_s;
struct ioDef_s;
typedef struct timerChDef_s {
    struct timerChRec_s *rec;
    const struct timerDef_s *timerDef;    // timer definition
    const struct ioDef_s *ioDef;          // io pin definition
    TIM_TypeDef *tim;                     // cache
    GPIO_TypeDef *gpio;                   // cache
    uint32_t pin;                         // cache
    uint8_t channel;
#ifdef STM32F303xC
    uint8_t pinAF;                        // pin Alternate Function multiplexer for this timer
#endif
} timerChDef_t;

typedef struct timerDef_s {
    struct timerRec_s* rec;
    TIM_TypeDef *tim;
    uint8_t irqCC, irqUP;
    uint8_t channels;        // number of channels allocated for this timer
    bool outputsNeedEnable;  // advanced timers
} timerDef_t;

// TODO
extern const timerChDef_t* const timerChannelMap[USABLE_IO_CHANNEL_COUNT];

void timerChConfigIC(const timerChDef_t *timCh, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(const timerChDef_t *timCh, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(const timerChDef_t *timCh, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerChDef_t *timCh);
volatile timCCR_t* timerChCCRLo(const timerChDef_t *timCh);
volatile timCCR_t* timerChCCRHi(const timerChDef_t *timCh);
volatile timCNT_t* timerChCNT(const timerChDef_t *timCh);
void timerChConfigOC(const timerChDef_t *timCh, bool outEnable, bool stateHigh);
void timerChConfigGPIO(const timerChDef_t *timCh, GPIO_Mode mode);

void timerCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerChDef_t *timCh, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(const timerChDef_t *timCh, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(const timerChDef_t *timCh, FunctionalState newState);
void timerChITConfig(const timerChDef_t *timCh, FunctionalState newState);
void timerChClearCCFlag(const timerChDef_t *timCh);

void timerChInit(const timerChDef_t *timCh, resourceOwner_t owner, resourceType_t resources, int irqPriority, uint16_t period, int timerFrequency);

//TODO:
resourceType_t timerChGetUsedResources(const timerChDef_t *timCh);
struct timerChRec_s* timerChRec(const timerChDef_t *timCh);
struct timerChRec_s* timerChRecDual(const timerChDef_t *timCh);

void timerInit(void);
void timerStart(void);
void timerForceOverflow(const struct timerDef_s *timDef);

