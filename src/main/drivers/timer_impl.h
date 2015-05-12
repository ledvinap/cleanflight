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

#define CC_CHANNELS_PER_TIMER 4              // TIM_Channel_1..4

typedef struct timerChRec_s {
    timerCCHandlerRec_t *edgeCallback;
    timerOvrHandlerRec_t *overflowCallback;
    resourceOwner_t owner;
} timerChRec_t;

typedef struct timerRec_s {
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
    uint32_t runningTime;   // accumulated time on last overflow
    uint16_t forcedTimerOverflowSkipped;
    uint8_t priority;
    uint8_t runningTimeEnabled;
    // channels are last - it is possible to partially avoid allocation of unused channels in future
    struct timerChRec_s channel[CC_CHANNELS_PER_TIMER];
} timerRec_t;

// declared here, defined in target
extern const timerChDef_t timerQueueChannelDef;

void timerInitTarget(void);
