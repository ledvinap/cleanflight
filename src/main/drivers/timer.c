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
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "nvic.h"

#include "gpio.h"
#include "system.h"
#include "callback.h"
#include "pin_debug.h"

#include "timer.h"
#include "timer_impl.h"

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4              // TIM_Channel_1..4

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
    uint32_t runningTime;   // accumulated time on last overflow
    uint16_t forcedTimerOverflowSkipped;
    uint8_t priority;
    uint8_t runningTimeEnabled;
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT];

typedef struct {
    channelType_t type;
    channelResources_t resourcesUsed;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[USABLE_IO_CHANNEL_COUNT];


static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim);

// return index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_SHF 10           // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

// let gcc do the work, switch should be quite optimized
    switch((unsigned)tim >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(1)
        _CASE(1);
#endif
#if USED_TIMERS & TIM_N(2)
        _CASE(2);
#endif
#if USED_TIMERS & TIM_N(3)
        _CASE(3);
#endif
#if USED_TIMERS & TIM_N(4)
        _CASE(4);
#endif
#if USED_TIMERS & TIM_N(8)
        _CASE(8);
#endif
#if USED_TIMERS & TIM_N(15)
        _CASE(15);
#endif
#if USED_TIMERS & TIM_N(16)
        _CASE(16);
#endif
#if USED_TIMERS & TIM_N(17)
        _CASE(17);
#endif
    default:  return ~1;  // make sure final index is out of range
    }
#undef _CASE
#undef _CASE_
}

TIM_TypeDef * const usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) TIM##i

#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

struct {
    uint8_t irqCC, irqUP;
} timerInfoConst[USED_TIMER_COUNT] = {
#define _DEF(i, iCC, iUP) {.irqCC = iCC, .irqUP = iUP}

#if USED_TIMERS & TIM_N(1)
    _DEF(1, TIM1_CC_IRQn, TIM1_UP_IRQn),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2, TIM2_IRQn, TIM2_IRQn),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3, TIM3_IRQn, TIM3_IRQn),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4, TIM4_IRQn, TIM4_IRQn),
#endif
#if USED_TIMERS & TIM_N(8)
# if defined(STM32F10X_XL)
    _DEF(8, TIM8_CC_IRQn, TIM8_UP_TIM13_IRQn),
# else // f10x_hd, f30x
    _DEF(8, TIM8_CC_IRQn, TIM8_UP_IRQn),
# endif
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15, TIM1_BRK_TIM15_IRQn, TIM1_BRK_TIM15_IRQn),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16, TIM1_UP_TIM16_IRQn, TIM1_UP_TIM16_IRQn),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17, TIM1_TRG_COM_TIM17_IRQn, TIM1_TRG_COM_TIM17_IRQn),
#endif
#undef _DEF
};

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

void timerConfigureIRQ(TIM_TypeDef *tim, uint8_t irqPriority)
{
    int timIdx = lookupTimerIndex(tim);
    if(timerConfig[timIdx].priority == 0xff)
        TIM_Cmd(usedTimers[timIdx],  ENABLE);

    if(irqPriority < timerConfig[timIdx].priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = timerInfoConst[timIdx].irqCC;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        if(timerInfoConst[timIdx].irqUP != timerInfoConst[timIdx].irqCC) {
            // two interrupts. We should check if another TIM is affected and maybe use different priority for UP
            // but now simply set priority to same value
             NVIC_InitStructure.NVIC_IRQChannel = timerInfoConst[timIdx].irqUP;
             NVIC_Init(&NVIC_InitStructure);
        }
        timerConfig[timIdx].priority = irqPriority;
    }
}

// TODO - this needs some refactoring
void timerConfigure(TIM_TypeDef *tim, uint8_t irqPriority, uint16_t period, int frequency)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    // TODO - we should special-case reiniitialization
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / frequency) - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    timerConfigureIRQ(tim, irqPriority);  // this will also enable timerc if neccesary
}

void timerCountRunningTime(TIM_TypeDef *tim, bool enable, uint8_t irqPriority)
{
    int timIdx = lookupTimerIndex(tim);
    timerConfigureIRQ(tim, irqPriority);
    timerConfig[timIdx].runningTimeEnabled = enable;
    // enable/disable overflow IRQ as necessary
    timerChConfig_UpdateOverflow(&timerConfig[timIdx], tim);
}

uint32_t timerGetRunningTimeBase(TIM_TypeDef *tim)
{
    int timIdx = lookupTimerIndex(tim);
    return timerConfig[timIdx].runningTime;
}

uint32_t timerGetRunningTimeCNT(TIM_TypeDef *tim)
{
    int timIdx = lookupTimerIndex(tim);
    uint32_t cnt;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is unhandled update event pending. Read CNT again to be sure that cnt is value after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += timerConfig[timIdx].runningTime + tim ->ARR + 1 - timerConfig[timIdx].forcedTimerOverflowSkipped;
        } else {
            // runningTime does match CNT captured
            cnt += timerConfig[timIdx].runningTime;
        }
    }
    return cnt;
}

// same as above, but assumes that timer has ARR=0xffff and no skip is performed
// TODO - f303 timer allows reading overflow bit with count, so ATOMIC_BLOCK can be shorter
uint32_t timerGetRunningTimeCNTfast(TIM_TypeDef *tim)
{
    int timIdx = lookupTimerIndex(tim);
    uint32_t cnt;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is unhandled update event pending. Read CNT again to be sure that value was read after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += timerConfig[timIdx].runningTime + 0x10000;
        } else {
            // runningTime does match CNT captured
            cnt += timerConfig[timIdx].runningTime;
        }
    }
    return cnt;
}

// allocate and configure timer channel. Timer priority is set to highest priority of its channels
// caller must check if channel is free
void timerChInit(const timerHardware_t *timHw, channelType_t type, channelResources_t resources, int irqPriority, uint16_t timerPeriod, int timerFrequency)
{
    unsigned channel = timHw - timerHardware;
    if(channel >= USABLE_IO_CHANNEL_COUNT)
        return;

    if(type != TYPE_FREE)   // pass TYPE_FREE to keep old owner
        timerChannelInfo[channel].type = type;

    timerChannelInfo[channel].resourcesUsed |= resources;

    if(!timHw->tim) return;  // TODO - not timer channel

    if(resources & RESOURCE_TIMER_DUAL) {
        // we must mark other channel too
        unsigned dualChannel = timerChFindDualChannel(timHw) - timerHardware;
        if(dualChannel < USABLE_IO_CHANNEL_COUNT)
            timerChannelInfo[dualChannel].resourcesUsed |= RESOURCE_TIMER;
    }

    unsigned timer = lookupTimerIndex(timHw->tim);
    if(timer >= USED_TIMER_COUNT)
        return;
    timerConfigure(timHw->tim, irqPriority, timerPeriod, timerFrequency);
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim) {
    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for(int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
            if(cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
        *chain = NULL;
    }
    // enable or disable IRQ
    TIM_ITConfig(tim, TIM_IT_Update, (cfg->overflowCallbackActive || cfg->runningTimeEnabled) ? ENABLE : DISABLE);
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if(edgeCallback == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    // enable channel IRQ
    if(edgeCallback)
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint16_t chLo = timHw->channel & ~TIM_Channel_2;   // lower channel
    uint16_t chHi = timHw->channel | TIM_Channel_2;    // upper channel
    uint8_t channelIndex = lookupChannelIndex(chLo);   // get index of lower channel

    if(edgeCallbackLo == NULL)   // disable irq before changing setting callback to NULL
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chLo), DISABLE);
    if(edgeCallbackHi == NULL)   // disable irq before changing setting callback to NULL
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chHi), DISABLE);

    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    // enable channel IRQs
    if(edgeCallbackLo) {
        TIM_ClearFlag(timHw->tim, TIM_IT_CCx(chLo));
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chLo), ENABLE);
    }
    if(edgeCallbackHi) {
        TIM_ClearFlag(timHw->tim, TIM_IT_CCx(chHi));
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chHi), ENABLE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
}

// enable or disable IRQ
void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), newState);
}

// clear Compare/Capture flag for channel
void timerChClearCCFlag(const timerHardware_t *timHw)
{
    TIM_ClearFlag(timHw->tim, TIM_IT_CCx(timHw->channel));
}

// configure timer channel GPIO mode
void timerChConfigGPIO(const timerHardware_t *timHw, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = timHw->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(timHw->gpio, &cfg);
}

// calculate input filter constant
// TODO - we should probably setup DTS to higher value to allow reasonable input filtering
//   - notice that prescaler[0] does use DTS for sampling - the sequence won't be monotonous anymore
static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,                 // fDTS !
        1*2, 1*4, 1*8,       // fCK_INT
        2*6, 2*8,            // fDTS/2
        4*6, 4*8,
        8*6, 8*8,
        16*5, 16*6, 16*8,
        32*5, 32*6, 32*8
    };
    for(unsigned i = 1; i < ARRAYLEN(ftab); i++)
        if(ftab[i] > ticks)
            return i - 1;
    return 0x0f;
}

// Configure input captupre
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
void timerChConfigICDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    // configure direct channel
    TIM_ICStructInit(&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel = timHw->channel ^ TIM_Channel_2;   // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    timCCER_t tmpccer = timHw->tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P << timHw->channel);
    tmpccer |= polarityRising ? (TIM_ICPolarity_Rising << timHw->channel) : (TIM_ICPolarity_Falling << timHw->channel);
    timHw->tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel | TIM_Channel_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel & ~TIM_Channel_2));
}


volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + timHw->channel);
}

volatile timCCR_t* timerChCNT(const timerHardware_t *timHw)
{
    return &timHw->tim->CNT;
}

void timerChConfigOC(const timerHardware_t *timHw, bool outEnable, bool activeHigh)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

// StdPeriph_Driver is so confusing - when switching mode, some bits are left from input mode
// it's probably best to rewrite this code to use direct register access

    TIM_OCStructInit(&TIM_OCInitStructure);
    if(outEnable) {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = activeHigh ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
    } else {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    }

    switch (timHw->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        TIM_ClearOC1Ref(timHw->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        TIM_ClearOC2Ref(timHw->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        TIM_ClearOC3Ref(timHw->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        TIM_ClearOC4Ref(timHw->tim, TIM_OCClear_Disable);
        break;
    }
}

const timerHardware_t* timerChFindDualChannel(const timerHardware_t *timHw)
{
    for(unsigned i = 0; i < USABLE_IO_CHANNEL_COUNT; i++)
        if(timerHardware[i].tim == timHw->tim
           && timerHardware[i].channel == (timHw->channel ^ TIM_Channel_2))
            return &timerHardware[i];
    return NULL;
}

channelResources_t timerChGetUsedResources(const timerHardware_t *timHw)
{
    unsigned channel = timHw - timerHardware;
    if(channel < USABLE_IO_CHANNEL_COUNT)
        return timerChannelInfo[channel].resourcesUsed;
    return 0;
}

static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;
#if 1
    while(tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch(bit) {
        case __builtin_clz(TIM_IT_Update): {
            uint16_t capture;
            capture = tim->ARR;
            // compensate skipped value if overflow was forced
            capture -= timerConfig->forcedTimerOverflowSkipped;
            timerConfig->forcedTimerOverflowSkipped = 0;
            // accumulate total time timer is running
            timerConfig->runningTime += capture + 1;
            timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
            while(cb) {
                cb->fn(cb, capture);
                cb = cb->next;
            }
            break;
        }
        case __builtin_clz(TIM_IT_CC1):
            timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
            break;
        case __builtin_clz(TIM_IT_CC2):
            timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
            break;
        case __builtin_clz(TIM_IT_CC3):
            timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
            break;
        case __builtin_clz(TIM_IT_CC4):
            timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
            break;
        }
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim->SR = ~TIM_IT_Update;
        uint16_t capture = tim->ARR;
        // compensate skipped value if overflow was forced
        capture -= timerConfig->forcedTimerOverflowSkipped;
        timerConfig->forcedTimerOverflowSkipped = 0;
        // accumulate total time timer is running
        timerConfig->runningTime += capture + 1;
        timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
        while(cb) {
            cb->fn(cb, capture);
            cb = cb->next;
        }
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim->SR = ~TIM_IT_CC1;
        timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim->SR = ~TIM_IT_CC2;
        timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[1], tim->CCR2);
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim->SR = ~TIM_IT_CC3;
        timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim->SR = ~TIM_IT_CC4;
        timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
    }
#endif
}
#if 1
// handler for shared interrupts when both timers need to check status bits
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TIM ## j, &timerConfig[TIMER_INDEX(j)]);          \
        pinDbgLo(DBP_TIMER);                                            \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        pinDbgLo(DBP_TIMER);                                            \
    } struct dummy
#else
// this will create histogram with timer durations
#define _TIM_IRQ_HANDLER(name, i)                                      \
    uint32_t dbghist_##name[32];                                       \
    void name(void)                                                    \
    {                                                                  \
        uint16_t start = TIM1->CNT;                                      \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);         \
        start = TIM1->CNT - start;                                         \
        if(start > 31) start = 31;                                         \
        dbghist_##name[start]++;                                       \
    } struct dummy
#endif

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
# if defined(STM32F10X)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);       // timer can't be shared
# endif
# ifdef STM32F303xC
#  if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, 1, 16);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 1);       // timer16 is not used
#  endif
# endif
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif
#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
# if defined(STM32F10X_XL)
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);
# else  // f10x_hd, f30x
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
# endif
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if defined(STM32F303xC) && ((USED_TIMERS & (TIM_N(1)|TIM_N(16))) == (TIM_N(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16);    // only timer16 is used, not timer1
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));
    // call target-specific initialization routine (enable peripheral clocks, etc)
    timerInitTarget();

#ifdef STM32F303xC
    for (uint8_t timerIndex = 0; timerIndex < USABLE_IO_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        if(timerHardwarePtr->alternateFunction != 0xff)
            GPIO_PinAFConfig(timerHardwarePtr->gpio, (uint16_t)timerHardwarePtr->gpioPinSource, timerHardwarePtr->alternateFunction);
    }
#endif

// initialize timer channel structures
    for(int i = 0; i < USABLE_IO_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
        timerChannelInfo[i].resourcesUsed = 0;
    }
    for(int i = 0; i < USED_TIMER_COUNT; i++) {
        timerConfig[i].priority = ~0;
    }
#ifdef PINDEBUG
// allocate debug pins here
    for(int i=0; i < USABLE_IO_CHANNEL_COUNT; i++) {
        if(pinDebugIsPinUsed(timerHardware[i].gpio, timerHardware[i].pin)) {
            // true pin allocation should be used when implemented
            timerChannelInfo[i].type = TYPE_PINDEBUG;
            timerChannelInfo[i].resourcesUsed = RESOURCE_OUTPUT;
        }
    }
#endif
#ifdef TIME_USE_TIMER
    // initialize timer used for timing functions
    timerConfigure(TIME_TIMER, NVIC_PRIO_TIME_TIMER, 0, 1000000);
    timerCountRunningTime(TIME_TIMER, true, NVIC_PRIO_TIME_TIMER);
#endif
}

// finish configuring timers after allocation phase
// start timers
// TODO - Work in progress - initialization routine must be modified/verified to start correctly without timers
void timerStart(void)
{
#if 0
    for(unsigned timer = 0; timer < USED_TIMER_COUNT; timer++) {
        int priority = -1;
        int irq = -1;
        for(unsigned hwc = 0; hwc < USABLE_IO_CHANNEL_COUNT; hwc++)
            if((timerChannelInfo[hwc].type != TYPE_FREE) && (timerHardware[hwc].tim == usedTimers[timer])) {
                // TODO - move IRQ to timer info
                irq = timerHardware[hwc].irq;
            }
        // TODO - aggregate required timer paramaters
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);
        if(priority >= 0) {  // maybe none of the channels was configured
            NVIC_InitTypeDef NVIC_InitStructure;

            NVIC_InitStructure.NVIC_IRQChannel = irq;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPLIT_PRIORITY_BASE(priority);
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_SPLIT_PRIORITY_SUB(priority);
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
        }
    }
#endif
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const TIM_TypeDef *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        uint16_t cnt=tim->CNT;
        // restart only if previous overflow was already handled
        // also don't modify forcedTimerOverflowSkipped
        if(!(tim->SR & TIM_IT_Update)) {
            timerConfig[timerIndex].forcedTimerOverflowSkipped = tim->ARR - cnt;
            // Force an overflow by setting the UG bit
            tim->EGR |= TIM_EGR_UG;
        }
    }
}

#ifdef TIME_USE_TIMER

uint32_t micros(void)
{
    return timerGetRunningTimeCNTfast(TIME_TIMER);
}

uint32_t millis(void)
{
    static uint32_t millisCounter = 0;
    static uint32_t microsLast = 0;

    uint32_t microsNow = micros();
    uint32_t delta = microsNow - microsLast;

    if(delta>1000) {
        delta /= 1000;
        millisCounter += delta;
        microsLast += delta * 1000;
    }
    return millisCounter;
}

static int32_t cmp32(uint32_t a, uint32_t b)
{
    return a-b;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t end = micros() + us;
    while(cmp32(micros(), end) <  0);
}

void delay(uint32_t ms) {
#if 0
    // corect implementation, wait 60 seconds each time
    while(ms > 60000) {
        delayMicroseconds(60000 * 1000);
        ms -= 60000 * 1000;
    }
#else
    // something is terribly broken if delay is longer than 15s. Maybe we should even trigger alarm
    if(ms > 15000)
        ms = 15000;
#endif
    delayMicroseconds(ms * 1000);
}

#endif
