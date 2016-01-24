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

#include "common/utils.h"
#include "io.h"
#include "timer.h"

// timer channel definition
typedef struct timerChDef_s {
    unsigned timerIdx : 5;               // index of timer
    unsigned channelIdx : 3;             // channel number (1-4 only)
    ioTag_t ioTag;                       // IO pin TAG (packed GPIO/PIN)
#ifdef STM32F303xC
    unsigned pinAF : 4;                  // alternate function
#endif
} timerChDef_t;

// timer definition
typedef struct timerDef_s {
    TIM_TypeDef *tim;                    // TIMx
    uint8_t irqCC, irqUP;                // CompareCapture and Update IRQn
    uint8_t channels;                    // number of channels allocated for this timer
    bool outputsNeedEnable;              // advanced timers
    rccPeriphTag_t rcc;                  // RCC bit to enable this timer (IO RCC is handled in IO)
} timerDef_t;

// record for timer channel
// initialized from DEF, unique for timer/channel combination
typedef struct timerChRec_s {
    timerRec_t *timRec;                       // timer rec
    TIM_TypeDef *tim;                         // timer peripheral, cache
    IO_t io;                                  // io pin for this timer
#ifdef STM32F303xC
    uint8_t pinAF;                            // pin Alternate Function multiplexer for GPIO pin
#endif
    uint8_t channel;                          // TIM_Channel_x, only 8 bits are used
    timerCCHandlerRec_t *edgeCallback;        // invoked on CC
    timerOvrHandlerRec_t *overflowCallback;   // only used to build Update callback list
    resourceOwner_t owner;
} timerChRec_t;

// data specific for timer
typedef struct timerRec_s {
    TIM_TypeDef *tim;                               // timer peripheral, cache
    timerOvrHandlerRec_t *overflowCallbackActive;   // NULL-terminated linkded list of active overflow callbacks (rebuild from channels)
    uint32_t runningTime;                           // accumulated ticks on last overflow. Must be enabled to work
    uint16_t forcedTimerOverflowSkipped;            // amount of ticks skipped when forcing overflow
    uint8_t priority;
    uint8_t runningTimeEnabled;
    uint8_t channels;                               // number of allocated channels
    const timerDef_t* def;                          // constant part to avoid caching everything
    // channels are last - only space up to last used channel is allocated. Outer structure contain members to allocate required space
    struct timerChRec_s channel[0];
} timerRec_t;

// pointer to timers used on target
extern timerRec_t * const timerRecPtrs[];
// PWM channels defined for channel
extern const timerChDef_t timerChannelMap[];
// channel used for timer queue (target specific. May be undefined if systick is used instead)
extern const timerChDef_t timerQueueChDef;

// decouple timerChannelMap length from rest of code
int timerChannelMap_Count(void);
