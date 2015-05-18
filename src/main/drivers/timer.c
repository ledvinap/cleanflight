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
#include "drivers/io.h"

#include "timer.h"
#include "timer_impl.h"

// index for given channel
static inline uint8_t getChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

// ISR flag for given channel
#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))
#define TIM_CCER_CCxP(ch) (TIM_CCER_CC1P << (ch))

static void timerUpdateOverflowHandlers(const timerDef_t *timDef);

// Configure timer IRQ priority (and enable timer now)
// Overflow and compare priorituies are the same now, TODO
void timerConfigureIRQ(const timerDef_t *timDef, uint8_t irqPriority)
{
    // enable timer if this if first call
    if(timDef->rec->priority == 0x00)   // TODO - hack to avoid initialization
        timDef->rec->priority = 0xff;

    if(timDef->rec->priority == 0xff)
        TIM_Cmd(timDef->tim,  ENABLE);

    if(irqPriority < timDef->rec->priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = timDef->irqCC;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        if(timDef->irqUP != timDef->irqCC) {
            // two interrupts. We should check if another TIM is affected and maybe use different priority for UP
            // but now simply set priority to same value
             NVIC_InitStructure.NVIC_IRQChannel = timDef->irqUP;
             NVIC_Init(&NVIC_InitStructure);
        }
        timDef->rec->priority = irqPriority;
    }
}

// TODO - this needs some refactoring
// priority is both for overflow and compare
// period is actual period, 0x10000 is passed as 0
// frequenct in Hz
void timerConfigure(const timerDef_t *timDef, uint8_t irqPriority, uint16_t period, int frequency)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_ClockCmd(timDef->rcc, ENABLE);

    // TODO - we should special-case reiniitialization
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xffff; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / frequency) - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timDef->tim, &TIM_TimeBaseStructure);

    timerConfigureIRQ(timDef, irqPriority);  // this will also enable timer if neccesary
}

// set state of timer running time calculation.
// When enabled, timer period is accumulated on each overflow in 32bit register
// oneshot time skips are accounted for
void timerCountRunningTime(const timerDef_t *timDef, bool enable, uint8_t irqPriority)
{
    timerConfigureIRQ(timDef, irqPriority);
    timDef->rec->runningTimeEnabled = enable;
    // enable/disable overflow IRQ as necessary
    timerUpdateOverflowHandlers(timDef);
}

// return actual accumulated running time on last overflow
uint32_t timerGetRunningTimeBase(const timerDef_t *timDef)
{
    return timDef->rec->runningTime;
}

// return actual running time - accumulated + CNT
// TODO - this may be special-cased according based on current BASEPRI, avoiding atomic block
uint32_t timerGetRunningTimeCNT(const timerDef_t *timDef)
{
    uint32_t cnt;
    TIM_TypeDef *tim = timDef->tim;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is unhandled update event pending. Read CNT again to be sure that cnt is value after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += timDef->rec->runningTime + tim->ARR + 1 - timDef->rec->forcedTimerOverflowSkipped;
        } else {
            // runningTime does match CNT captured
            cnt += timDef->rec->runningTime;
        }
    }
    return cnt;
}

// same as above, but assumes that timer has ARR=0xffff and no skip is performed
// this saves some cycles. This function is used for delay and time functions, so every cycle counts ..
// TODO - f303 timer allows reading overflow bit with count, so ATOMIC_BLOCK can be shorter
// TODO - check basepri, use two different code paths instead of locking interrupts
uint32_t timerGetRunningTimeCNTfast(const timerDef_t *timDef)
{
    uint32_t cnt;
    TIM_TypeDef *tim = timDef->tim;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is unhandled update event pending. Read CNT again to be sure that value was read after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += timDef->rec->runningTime + 0x10000;
        } else {
            // runningTime does match CNT captured
            cnt += timDef->rec->runningTime;
        }
    }
    return cnt;
}

// allocate and configure timer channel. Timer priority is set to highest priority of its channels
// caller must check if channel is free
// TODO - resourceType
void timerChInit(const timerChDef_t *timChDef, resourceOwner_t owner, resourceType_t resources, int irqPriority, uint16_t timerPeriod, int timerFrequency)
{
    const ioDef_t* ioDef = timChDef->ioDef;
    if(ioDef) {  // claim io channel
        if(owner != OWNER_FREE)   // pass OWNER_FREE to keep old owner
            ioDef->rec->owner = owner;

    }

    timChDef->rec->owner = owner;
    if(resources & RESOURCE_TIMER_DUAL) {
        // we must mark other channel too
        timerChRec_t *dualRec = timerChRecDual(timChDef);
        if(dualRec)
            dualRec->owner = owner;
    }

    timerConfigure(timChDef->timerDef, irqPriority, timerPeriod, timerFrequency);
}

void timerCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerUpdateOverflowHandlers(const timerDef_t *timDef)
{
    timerRec_t *timRec = timDef->rec;
    timerOvrHandlerRec_t **chain = &timRec->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for(int i = 0; i < timDef->channels; i++)
            if(timRec->channel[i].overflowCallback) {
                *chain = timRec->channel[i].overflowCallback;
                chain = &timRec->channel[i].overflowCallback->next;
            }
        *chain = NULL;
    }
    // enable or disable IRQ
    TIM_ITConfig(timDef->tim, TIM_IT_Update, (timRec->overflowCallbackActive || timRec->runningTimeEnabled) ? ENABLE : DISABLE);
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerChDef_t *timChDef, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    timerChRec_t *timChRec = timChDef->rec;

    if(edgeCallback == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(timChDef->channel), DISABLE);
    // setup callback info
    timChRec->edgeCallback = edgeCallback;
    timChRec->overflowCallback = overflowCallback;
    // enable channel IRQ
    if(edgeCallback)
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(timChDef->channel), ENABLE);

    timerUpdateOverflowHandlers(timChDef->timerDef);
}

// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
void timerChConfigCallbacksDual(const timerChDef_t *timChDef, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint16_t loCh = timChDef->channel & ~TIM_Channel_2;   // lower channel
    uint16_t hiCh = timChDef->channel | TIM_Channel_2;    // upper channel

    timerChRec_t *loRec, *hiRec;
    if(timChDef->channel & TIM_Channel_2)  { // we are called with upper channel
        hiRec = timerChRec(timChDef);
        loRec = timerChRecDual(timChDef);
    } else {
        loRec = timerChRec(timChDef);
        hiRec = timerChRecDual(timChDef);
    }

    if(edgeCallbackLo == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(loCh), DISABLE);
    if(edgeCallbackHi == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(hiCh), DISABLE);

    // setup callback info
    loRec->edgeCallback = edgeCallbackLo;
    hiRec->edgeCallback = edgeCallbackHi;
    loRec->overflowCallback = overflowCallback;
    hiRec->overflowCallback = NULL;

    // enable channel IRQs
    if(edgeCallbackLo) {
        TIM_ClearFlag(timChDef->tim, TIM_IT_CCx(loCh));
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(loCh), ENABLE);
    }
    if(edgeCallbackHi) {
        TIM_ClearFlag(timChDef->tim, TIM_IT_CCx(hiCh));
        TIM_ITConfig(timChDef->tim, TIM_IT_CCx(hiCh), ENABLE);
    }

    timerUpdateOverflowHandlers(timChDef->timerDef);
}

// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(const timerChDef_t *timChDef, FunctionalState newState) {
    TIM_ITConfig(timChDef->tim, TIM_IT_CCx(timChDef->channel & ~TIM_Channel_2), newState);
}

// enable or disable IRQ
void timerChITConfig(const timerChDef_t *timChDef, FunctionalState newState)
{
    TIM_ITConfig(timChDef->tim, TIM_IT_CCx(timChDef->channel), newState);
}

// clear Compare/Capture flag for channel
void timerChClearCCFlag(const timerChDef_t *timChDef)
{
    TIM_ClearFlag(timChDef->tim, TIM_IT_CCx(timChDef->channel));
}

// configure timer channel GPIO mode
void timerChConfigGPIO(const timerChDef_t *timChDef, GPIO_Mode mode)
{
    if(timChDef->ioDef) {
#ifdef STM32F303xC
        IOConfigGPIOAF(timChDef->ioDef, mode, timChDef->pinAF);
#else
        IOConfigGPIO(timChDef->ioDef, mode);
#endif
    }
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

// Configure input captupre on timer channel
// inputFilterTicks is converted from timer ticks to timer settings, resulting time is guaranteed to be shorter or equal to requested value
void timerChConfigIC(const timerChDef_t *timChDef, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timChDef->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timChDef->tim, &TIM_ICInitStructure);
}

// configure input channel for capture uing dual timers
// polarity is for Low channel (capture order is always Lo - Hi)
// inputFilterTicks is converted from timer ticks to timer settings, resulting time is guaranteed to be shorter or equal to requested value
void timerChConfigICDual(const timerChDef_t *timChDef, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timChDef->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    // configure direct channel
    TIM_ICStructInit(&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = timChDef->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit(timChDef->tim, &TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel = timChDef->channel ^ TIM_Channel_2;   // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit(timChDef->tim, &TIM_ICInitStructure);
}

// change input capture polarity (without dsiabling timer)
void timerChICPolarity(const timerChDef_t *timChDef, bool polarityRising)
{
    if(polarityRising) {
        timChDef->tim->CCER &= ~TIM_CCER_CCxP(timChDef->channel);
    } else {
        timChDef->tim->CCER |= TIM_CCER_CCxP(timChDef->channel);
    }
}

volatile timCCR_t* timerChCCRHi(const timerChDef_t *timChDef)
{
    return (volatile timCCR_t*)((volatile char*)&timChDef->tim->CCR1 + (timChDef->channel | TIM_Channel_2));
}

volatile timCCR_t* timerChCCRLo(const timerChDef_t *timChDef)
{
    return (volatile timCCR_t*)((volatile char*)&timChDef->tim->CCR1 + (timChDef->channel & ~TIM_Channel_2));
}


volatile timCCR_t* timerChCCR(const timerChDef_t *timChDef)
{
    return (volatile timCCR_t*)((volatile char*)&timChDef->tim->CCR1 + timChDef->channel);
}

volatile timCCR_t* timerChCNT(const timerChDef_t *timChDef)
{
    return &timChDef->tim->CNT;
}

// configure output compare on timer channel
void timerChConfigOC(const timerChDef_t *timChDef, bool outEnable, bool activeHigh)
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

    switch (timChDef->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(timChDef->tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timChDef->tim, TIM_OCPreload_Disable);
        TIM_ClearOC1Ref(timChDef->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(timChDef->tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timChDef->tim, TIM_OCPreload_Disable);
        TIM_ClearOC2Ref(timChDef->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(timChDef->tim, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timChDef->tim, TIM_OCPreload_Disable);
        TIM_ClearOC3Ref(timChDef->tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(timChDef->tim, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timChDef->tim, TIM_OCPreload_Disable);
        TIM_ClearOC4Ref(timChDef->tim, TIM_OCClear_Disable);
        break;
    }
}

timerChRec_t* timerChRecDual(const timerChDef_t *timChDef)
{
    if(timChDef->channel & TIM_Channel_2)
        return timChDef->rec - 1;  // high channel, low channel must be before it
    else
        return timChDef->rec + 1;  // low channel. Hi channel should follow if it does exist
}

timerChRec_t* timerChRec(const timerChDef_t *timChDef)
{
    return timChDef->rec; 
}

// TIMER_DUAL must be returned too TODO
resourceType_t timerChGetUsedResources(const timerChDef_t *timChDef)
{
    return timChDef->ioDef->rec->resourcesUsed;
}

// timer IRQ handler. Try to keep it as fast as possible (115200 baud on softserial shall be possbile, every instruction counts)
// this handler assumes that only IRQ's for existing, configured channels are invoked, no validation is done in this routine
static void timIRQHandler(TIM_TypeDef *tim, timerRec_t *timRec)
{
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;
#if 1
    while(tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handlers corectly
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
            capture -= timRec->forcedTimerOverflowSkipped;
            timRec->forcedTimerOverflowSkipped = 0;
            // accumulate total time timer is running (always count in update interrupt is enabled)
            timRec->runningTime += capture + 1;
            // invoke all active overflow handlers
            timerOvrHandlerRec_t *cb = timRec->overflowCallbackActive;
            while(cb) {
                cb->fn(cb, capture);
                cb = cb->next;
            }
            break;
        }
        case __builtin_clz(TIM_IT_CC1):
            timRec->channel[0].edgeCallback->fn(timRec->channel[0].edgeCallback, tim->CCR1);
            break;
        case __builtin_clz(TIM_IT_CC2):
            timRec->channel[1].edgeCallback->fn(timRec->channel[1].edgeCallback, tim->CCR2);
            break;
        case __builtin_clz(TIM_IT_CC3):
            timRec->channel[2].edgeCallback->fn(timRec->channel[2].edgeCallback, tim->CCR3);
            break;
        case __builtin_clz(TIM_IT_CC4):
            timRec->channel[3].edgeCallback->fn(timRec->channel[3].edgeCallback, tim->CCR4);
            break;
        }
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim->SR = ~TIM_IT_Update;
        uint16_t capture = tim->ARR;
        // compensate skipped value if overflow was forced
        capture -= timRec->forcedTimerOverflowSkipped;
        timRec->forcedTimerOverflowSkipped = 0;
        // accumulate total time timer is running
        timRec->runningTime += capture + 1;
        timerOvrHandlerRec_t *cb = timRec->overflowCallbackActive;
        while(cb) {
            cb->fn(cb, capture);
            cb = cb->next;
        }
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim->SR = ~TIM_IT_CC1;
        timRec->channel[0].edgeCallback->fn(timRec->channel[0].edgeCallback, tim->CCR1);
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim->SR = ~TIM_IT_CC2;
        timRec->channel[1].edgeCallback->fn(timRec->channel[1].edgeCallback, tim->CCR2);
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim->SR = ~TIM_IT_CC3;
        timRec->channel[2].edgeCallback->fn(timRec->channel[2].edgeCallback, tim->CCR3);
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim->SR = ~TIM_IT_CC4;
        timRec->channel[3].edgeCallback->fn(timRec->channel[3].edgeCallback, tim->CCR4);
    }
#endif
}
#if 1
// handler for shared interrupts when both timers need to check status bits
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timIRQHandler(DEFIO_TIM(DEFIO_TIM_ID__  ## i), &DEFIO_TIMER_REC(DEFIO_TIM_ID__  ## i)); \
        timIRQHandler(DEFIO_TIM(DEFIO_TIM_ID__  ## j), &DEFIO_TIMER_REC(DEFIO_TIM_ID__  ## j)); \
        pinDbgLo(DBP_TIMER);                                            \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timIRQHandler(DEFIO_TIM(DEFIO_TIM_ID__  ## i), &DEFIO_TIMER_REC(DEFIO_TIM_ID__  ## i)); \
        pinDbgLo(DBP_TIMER);                                            \
    } struct dummy
#else
// this will create histogram with timer durations
#define _TIM_IRQ_HANDLER(name, i)                                      \
    uint32_t dbghist_##name[32];                                       \
    void name(void)                                                    \
    {                                                                  \
        uint16_t start = TIM1->CNT;                                      \
        timIRQHandler(TIM ## i, &timerRecs[TIMER_INDEX(i)]);         \
        start = TIM1->CNT - start;                                         \
        if(start > 31) start = 31;                                         \
        dbghist_##name[start]++;                                       \
    } struct dummy
#endif

#ifndef USED_TIMERS
# error "USED_TIMERS must be defined"
#endif

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, TIM1);
# if defined(STM32F10X)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, TIM1);       // timer can't be shared
# endif
# if defined(STM32F303xC)
#  if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, TIM1, TIM16);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, TIM1);       // timer16 is not used
#  endif
# endif
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, TIM2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, TIM3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, TIM4);
#endif
#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, TIM8);
# if defined(STM32F10X_XL)
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, TIM8);
# else  // f10x_hd, f30x
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, TIM8);
# endif
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, TIM15);
#endif
#if defined(STM32F303xC) && ((USED_TIMERS & (TIM_N(1)|TIM_N(16))) == (TIM_N(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, TIM16);    // only timer16 is used, not timer1
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, TIM17);
#endif

void timerInit(void)
{
// TODO TIM
#if TODO
    memset(timerRecs, 0, sizeof (timerRecs));
    // call target-specific initialization routine (enable peripheral clocks, etc)
#endif
    timerInitTarget();
#if TODO
    for(int i = 0; i < USED_TIMER_COUNT; i++) {
        timerRecs[i].priority = ~0;
    }
#endif
#if defined(PINDEBUG) && 0
// allocate debug pins here TODO - move to IO
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
 * Skipped ticks are compensaded for in overflow handler
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(const timerDef_t *timDef)
{
    TIM_TypeDef * tim = timDef->tim;

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the skipped count so that PPM reading will work on the same timer that was forced to overflow
        uint16_t cnt = tim->CNT;
        // restart only if previous overflow was already handled
        // also don't modify forcedTimerOverflowSkipped
        if(!(tim->SR & TIM_IT_Update)) {
            timDef->rec->forcedTimerOverflowSkipped = tim->ARR - cnt;
            // Force an overflow by setting the UG bit
            tim->EGR |= TIM_EGR_UG;
        }
    }
}

#ifdef TIME_USE_TIMER

// retrun current time in microseconds
uint32_t micros(void)
{
    return timerGetRunningTimeCNTfast(TIME_TIMER);
}

// retrun current time in milliseconds
// this function must be called at least once per microsecond counter overflow (once per hour)
uint32_t millis(void)
{
    static volatile uint32_t millisCounter = 0;
    static volatile uint32_t microsLast = 0;

    uint32_t microsLastCache = microsLast;
    uint32_t microsNow = micros();
    uint32_t delta = microsNow - microsLastCache;

    if(delta > 1000) {
        // this cvalculation is executed at most once per millisecond
        delta /= 1000;
        ATOMIC_BLOCK(NVIC_PRIO_TIME_TIMER) {
            if(microsLastCache == microsLast) {
                // ok, we won the right to update counter
                millisCounter += delta;
                microsLast += delta * 1000;
            } else {
                ; // nop;  someone updated counter before we got here. Unless we spent whole millisecond since calculating delta, millisCounter must be correct
            }
        }
    }
    return millisCounter;
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
