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

struct timerChDef_s {
    unsigned timerIdx : 5;               // index of timer
    unsigned channelIdx : 3;             // channel number (1-4 only)
    ioTag_t ioTag;                       // IO pin TAG (packed GPIO/PIN)
#ifdef STM32F303xC
    unsigned pinAf : 4;                  // alternate function
#endif
};

struct timerDef_s {
    TIM_TypeDef *tim;
    uint8_t irqCC, irqUP;
    uint8_t channels;        // number of channels allocated for this timer
    bool outputsNeedEnable;  // advanced timers
    rccPeriphTag_t rcc;      // RCC bit to enable this timer (IO RCC is handled in IO)
};

// record for timer channel
// initialized from DEF, unique for timer/channel combination
struct timerChRec_s {
    timerRec_t *timRec;                  // timer rec
    TIM_TypeDef *tim;                    // timer peripheral, cache
    ioRec_t *ioRec;                      // io pin for this timer
#ifdef STM32F303xC
    uint8_t pinAF;                       // pin Alternate Function multiplexer for this timer (TODO - init only?)
#endif
    uint8_t channel;                     // TIM_Channel_x, only 8 bits are used
    timerCCHandlerRec_t *edgeCallback;
    timerOvrHandlerRec_t *overflowCallback;
    resourceOwner_t owner;
};

#ifdef STM32F303xC
# define DEFIO_TIMERCH_REC_INITIALIZER {0,0,0,0,0,0,0,0}
#else
# define DEFIO_TIMERCH_REC_INITIALIZER {0,0,0,0,0,0,0}
#endif

// data specific for timer
struct timerRec_s {
    TIM_TypeDef *tim;                               // timer peripheral, cache
    timerOvrHandlerRec_t *overflowCallbackActive;   // NULL-terminated linkded list of active overflow callbacks
    uint32_t runningTime;                           // accumulated time on last overflow
    uint16_t forcedTimerOverflowSkipped;            // time skipped on last forced overflow
    uint8_t priority;
    uint8_t runningTimeEnabled;
    uint8_t channels;                               // number of allocated channels
    const timerDef_t* def;                          // constant part to avoid caching everrything
    // channels are last - only space up to last used channel is allocated
    struct timerChRec_s channel[];
};

#define DEFIO_TIMER_REC_INITIALIZER {0,0,0,0,0,0,0,0}

extern timerRec_t * const timerRecPtrs[];
extern const timerChDef_t timerChannelMap[];

void timerInitTarget(void);
