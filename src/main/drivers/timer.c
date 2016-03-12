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

#include <platform.h>
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
#include "timer_def.h"

// index for given channel
static inline uint8_t getTIM_Channel_Index(uint16_t channel)
{
    return channel >> 2;
}

static inline uint16_t getTIM_Channel_x(int index) {
    return index << 2;
}

// ISR flag for given channel
#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))
#define TIM_CCER_CCxP(ch) (TIM_CCER_CC1P << (ch))

static void timerUpdateOverflowHandlers(timerRec_t *timRec);

// Configure timer IRQ priority (and enable timer now)
// Overflow and compare priorities are the same now, TODO
static void timerConfigureIRQ(timerRec_t *timRec, uint8_t irqPriority)
{
    const timerDef_t* def = timRec->def;
    // enable timer if this if first call
    if(timRec->priority == 0xff)   // TODO
        TIM_Cmd(timRec->tim,  ENABLE);

    if(irqPriority < timRec->priority) {
        // it would be better to set priority in the end, but current startup sequence is not ready
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = def->irqCC;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        if(def->irqUP != def->irqCC) {
            // two interrupts. We should check if another TIM is affected and maybe use different priority for UP
            // but now simply set priority to same value
            NVIC_InitStructure.NVIC_IRQChannel = def->irqUP;
            NVIC_Init(&NVIC_InitStructure);
        }
        timRec->priority = irqPriority;
    }
}

// TODO - this needs some refactoring
// priority is both for overflow and compare
// period is actual period, 0x10000 is maximum on supported HW
// frequenct in Hz
void timerConfigure(timerRec_t *timRec, uint8_t irqPriority, int period, int frequency)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    const timerDef_t *def = timRec->def;

    RCC_ClockCmd(def->rcc, ENABLE);

    // TODO - we should special-case reiniitialization
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xffff; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
   TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / frequency) - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timRec->tim, &TIM_TimeBaseStructure);

    timerConfigureIRQ(timRec, irqPriority);  // this will also enable timer if neccesary
}

// set state of timer running time calculation.
// When enabled, timer period is accumulated on each overflow in 32bit register
// oneshot time skips are accounted for
void timerCountRunningTime(timerRec_t *timRec, bool enable, uint8_t irqPriority)
{
    timerConfigureIRQ(timRec, irqPriority);
    timRec->runningTimeEnabled = enable;
    // enable/disable overflow IRQ as necessary
    timerUpdateOverflowHandlers(timRec);
}

// return actual accumulated running time on last overflow
uint32_t timerGetRunningTimeBase(timerRec_t *timRec)
{
    return timRec->runningTime;
}

// return actual running time - accumulated + CNT
// TODO - this may be special-cased according based on current BASEPRI, avoiding atomic block
uint32_t timerGetRunningTimeCNT(timerRec_t *timRec)
{
    uint32_t cnt;
    TIM_TypeDef *tim = timRec->tim;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        ATOMIC_BARRIER(timRec); // runningTime must be read after IRQs are disabled
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is update event pending (unhandled yet). Read CNT again to be sure that cnt is value after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += tim->ARR + 1 - timRec->forcedTimerOverflowSkipped;
        }
        cnt += timRec->runningTime;  // add time on last handled overflow
    }
    return cnt;
}

// same as above, but assumes that timer has ARR=0xffff and no skip is performed
// this saves some cycles. This function is used for delay and time functions, so every cycle counts ..
// TODO - f303 timer allows reading overflow bit with count, so ATOMIC_BLOCK can be shorter
// TODO - check basepri, use two different code paths instead of locking interrupts
uint32_t timerGetRunningTimeCNTfast(timerRec_t *timRec)
{
    uint32_t cnt;
    TIM_TypeDef *tim = timRec->tim;
    ATOMIC_BLOCK_NB(NVIC_PRIO_TIMER) {
        ATOMIC_BARRIER(timRec);                // runningTime must be read after IRQs are disabled
        cnt = tim->CNT;
        if(tim->SR & TIM_IT_Update) {
            // there is unhandled update event pending. Read CNT again to be sure that value was read after overflow
            cnt = tim->CNT;
            // and add overflow value manually
            cnt += 0x10000;
        }
        cnt += timRec->runningTime;       // add time on last handled overflow
    }
    return cnt;
}

// Allocate and configure timer channel. User must pass timerChDef_t* structure to specify timer channel and remap.
// Timer priority is set to highest priority of its channels
// caller must check if channel is free
// TODO - resourceType
timerChRec_t* timerChInit(const timerChDef_t *timChDef, resourceOwner_t owner, resourceType_t resources, int irqPriority, uint16_t timerPeriod, int timerFrequency)
{
    timerChRec_t *timChRec = timerChDef_TimChRec(timChDef);
    if(timChRec == NULL)
        return NULL;

    IO_t io = timerChDef_IO(timChDef);
    if(io) {  // claim io channel
        IOInit(io, owner, resources);
    }
    timChRec->io = io;
#ifdef STM32F303xC
    timChRec->pinAF = timChDef->pinAF;
#endif
    timChRec->owner = owner;
    if(resources & RESOURCE_TIMER_DUAL) {
        // we must mark other channel too
        timerChRec_t *dualRec = timerChRecDual(timChRec);
        if(dualRec)
            dualRec->owner = owner;
    }

    timerConfigure(timChRec->timRec, irqPriority, timerPeriod, timerFrequency);
    return timChRec;
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
// linked list is always build again
static void timerUpdateOverflowHandlers(timerRec_t *timRec)
{
    timerOvrHandlerRec_t **chain = &timRec->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for(int i = 0; i < timRec->channels; i++)
            if(timRec->channel[i].overflowCallback) {
                *chain = timRec->channel[i].overflowCallback;
                chain = &timRec->channel[i].overflowCallback->next;
            }
        *chain = NULL;
    }
    // enable or disable IRQ
    TIM_ITConfig(timRec->tim, TIM_IT_Update, (timRec->overflowCallbackActive || timRec->runningTimeEnabled) ? ENABLE : DISABLE);
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChConfigCallbacks(timerChRec_t *timChRec, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    if(edgeCallback == NULL)   // disable irq before changing callback to NULL
        TIM_ITConfig(timChRec->tim, TIM_IT_CCx(timChRec->channel), DISABLE);
    // setup callback info
    timChRec->edgeCallback = edgeCallback;
    timChRec->overflowCallback = overflowCallback;
    // enable channel IRQ
    if(edgeCallback)
        TIM_ITConfig(timChRec->tim, TIM_IT_CCx(timChRec->channel), ENABLE);

    timerUpdateOverflowHandlers(timChRec->timRec);
}

// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is passed
// This is intended for dual capture mode (each channel handles one transition)
void timerChConfigCallbacksDual(timerChRec_t *timChRec, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint16_t loCh = timChRec->channel & ~TIM_Channel_2;   // lower channel
    uint16_t hiCh = timChRec->channel | TIM_Channel_2;    // upper channel

    TIM_TypeDef *tim = timChRec->tim;
    timerChRec_t *loRec, *hiRec;
    if(timChRec->channel & TIM_Channel_2)  { // we are called with upper channel
        hiRec = timChRec;
        loRec = timerChRecDual(timChRec);
    } else {
        loRec = timChRec;
        hiRec = timerChRecDual(timChRec);
    }
    // disable irq before changing callbacks to NULL
    if(edgeCallbackLo == NULL)
        TIM_ITConfig(tim, TIM_IT_CCx(loCh), DISABLE);
    if(edgeCallbackHi == NULL)
        TIM_ITConfig(tim, TIM_IT_CCx(hiCh), DISABLE);

    // setup callback info
    loRec->edgeCallback = edgeCallbackLo;
    hiRec->edgeCallback = edgeCallbackHi;
    loRec->overflowCallback = overflowCallback;
    hiRec->overflowCallback = NULL;               // overflow only on one channel

    // enable channel IRQs
    if(edgeCallbackLo) {
        TIM_ClearFlag(tim, TIM_IT_CCx(loCh));
        TIM_ITConfig(tim, TIM_IT_CCx(loCh), ENABLE);
    }
    if(edgeCallbackHi) {
        TIM_ClearFlag(tim, TIM_IT_CCx(hiCh));
        TIM_ITConfig(tim, TIM_IT_CCx(hiCh), ENABLE);
    }

    timerUpdateOverflowHandlers(timChRec->timRec);
}

// enable/disable IRQ for low channel in dual configuration
void timerChITConfigDualLo(timerChRec_t *timChRec, FunctionalState newState) {
    TIM_ITConfig(timChRec->tim, TIM_IT_CCx(timChRec->channel & ~TIM_Channel_2), newState);
}

// enable or disable IRQ
void timerChITConfig(timerChRec_t *timChRec, FunctionalState newState)
{
    TIM_ITConfig(timChRec->tim, TIM_IT_CCx(timChRec->channel), newState);
}

// clear Compare/Capture flag for channel
void timerChClearCCFlag(timerChRec_t *timChRec)
{
    TIM_ClearFlag(timChRec->tim, TIM_IT_CCx(timChRec->channel));
}

// configure timer channel GPIO mode
void timerChConfigGPIO(timerChRec_t *timChRec, ioConfig_t config)
{
    if(timChRec->io) {
#ifdef STM32F303xC
        IOConfigGPIOAF(timChRec->io, config, timChRec->pinAF);
#else
        IOConfigGPIO(timChRec->io, config);
#endif
    }
}

void timerChIOWrite(timerChRec_t *timCh, bool value)
{
    IOWrite(timCh->io, value);
}

// calculate input filter value (for TIM_ICFilter)
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
void timerChConfigIC(timerChRec_t *timChRec, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timChRec->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timChRec->tim, &TIM_ICInitStructure);
}

// configure input channel for capture uing dual timers
// polarity is for Low channel (capture order is always Lo - Hi)
// inputFilterTicks is converted from timer ticks to timer settings, resulting time is guaranteed to be shorter or equal to requested value
void timerChConfigICDual(timerChRec_t *timChRec, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timChRec->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    // configure direct channel
    TIM_ICStructInit(&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = timChRec->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit(timChRec->tim, &TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel = timChRec->channel ^ TIM_Channel_2;   // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit(timChRec->tim, &TIM_ICInitStructure);
}

// change input capture polarity (without disabling timer)
void timerChICPolarity(timerChRec_t *timChRec, bool polarityRising)
{
    if(polarityRising)
        timChRec->tim->CCER &= ~TIM_CCER_CCxP(timChRec->channel);
    else
        timChRec->tim->CCER |= TIM_CCER_CCxP(timChRec->channel);
}

static volatile timCCR_t* timerChCCR_Channel(timerChRec_t *timChRec, unsigned channel)
{
    return (volatile timCCR_t*)((volatile char*)&timChRec->tim->CCR1 + channel);
}

volatile timCCR_t* timerChCCRHi(timerChRec_t *timChRec)
{
    return timerChCCR_Channel(timChRec, timChRec->channel | TIM_Channel_2);
}

volatile timCCR_t* timerChCCRLo(timerChRec_t *timChRec)
{
    return timerChCCR_Channel(timChRec, timChRec->channel & ~TIM_Channel_2);
}


volatile timCCR_t* timerChCCR(timerChRec_t *timChRec)
{
    return timerChCCR_Channel(timChRec, timChRec->channel);
}

volatile timCCR_t* timerChCNT(timerChRec_t *timChRec)
{
    return &timChRec->tim->CNT;
}

TIM_TypeDef* timerChTIM(timerChRec_t *timChRec)
{
    return timChRec->tim;
}

// configure output compare on timer channel
void timerChConfigOC(timerChRec_t *timChRec, bool outEnable, bool activeHigh)
{
    TIM_TypeDef *tim = timChRec->tim;
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

    switch (timChRec->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(tim, TIM_OCPreload_Disable);
        TIM_ClearOC1Ref(tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(tim, TIM_OCPreload_Disable);
        TIM_ClearOC2Ref(tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(tim, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(tim, TIM_OCPreload_Disable);
        TIM_ClearOC3Ref(tim, TIM_OCClear_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(tim, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(tim, TIM_OCPreload_Disable);
        TIM_ClearOC4Ref(tim, TIM_OCClear_Disable);
        break;
    }
    if (outEnable && timChRec->timRec->def->outputsNeedEnable)
        TIM_CtrlPWMOutputs(tim, ENABLE);
}

void timerChConfigOCPwm(timerChRec_t *timChRec, uint16_t value)
{
    TIM_TypeDef *tim = timChRec->tim;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (timChRec->channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
    if (timChRec->timRec->def->outputsNeedEnable)
        TIM_CtrlPWMOutputs(tim, ENABLE);
}


timerChRec_t* timerChRecDual(timerChRec_t *timChRec)
{
    if(timChRec->channel & TIM_Channel_2)
        return timChRec - 1;  // high channel, low channel is before it in memory
    else
        return timChRec + 1;  // low channel, hi channel follows if it does exist
}

// TODO - resource allocation needs improvement
resourceType_t timerChGetResources(timerChRec_t *timChRec)
{
    resourceType_t resources = 0;
    IO_t io = timChRec->io;
    if(io)
        resources |= IOGetResources(io);
    IO_t ioDual = timerChRecDual(timChRec)->io;
    if(ioDual && (IOGetResources(ioDual) & RESOURCE_TIMER_DUAL))
        resources |=  IOGetResources(ioDual);
    return resources;
}

TIM_TypeDef* timerChDef_TIM(const timerChDef_t* timChDef)
{
    timerChRec_t * timChRec = timerChDef_TimChRec(timChDef);
    if(!timChRec) return NULL;
    return timChRec->tim;
}

IO_t timerChDef_IO(const timerChDef_t* timChDef)
{
    return IOGetByTag(timChDef->ioTag);
}

// TODO - what if caller wants old channel resources?
// TODO - resource-dual must be associated with timer channel, not IO
resourceType_t timerChDef_GetResources(const timerChDef_t* timChDef)
{
    resourceType_t resources = 0;
    IO_t io = timerChDef_IO(timChDef);
    if(io)
        resources |= IOGetResources(io);
    timerChRec_t *timChRec = timerChDef_TimChRec(timChDef);
    resources |= timerChGetResources(timChRec);
    return resources;
}

timerChRec_t* timerChDef_TimChRec(const timerChDef_t* timChDef)
{
    timerRec_t * timRec = timerChDef_TimRec(timChDef);
    if(timRec == NULL)
        return NULL;
    if(timChDef->channelIdx >= timRec->channels)
        return NULL;
    return &timRec->channel[timChDef->channelIdx];
}

timerRec_t* timerChDef_TimRec(const timerChDef_t* timChDef)
{
    return timerRecPtrs[timChDef->timerIdx];
}

// timer IRQ handler. Try to keep it as fast as possible (115200 baud on softserial shall be possbile, every instruction counts)
// this handler assumes that only IRQ's for existing, configured channels are invoked, no validation is done in this routine
static void timIRQHandler(TIM_TypeDef *tim, timerRec_t *timRec)
{
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;     // ignore flags for which IRQ is not enabled
#if 1   // switch based implementation
    while(tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handlers corectly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch(bit) {
        case __builtin_clz(TIM_IT_Update): {
            uint16_t period;
            period = tim->ARR;
            // compensate skipped value if overflow was forced
            period -= timRec->forcedTimerOverflowSkipped;
            timRec->forcedTimerOverflowSkipped = 0;
            // accumulate total time timer is running (always count if update interrupt is enabled)
            timRec->runningTime += period + 1;
            // invoke all active overflow handlers (aranged in linked list)
            timerOvrHandlerRec_t *cb = timRec->overflowCallbackActive;
            while(cb) {
                cb->fn(cb, period);
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
        uint16_t period = tim->ARR;
        // compensate skipped value if overflow was forced
        period -= timRec->forcedTimerOverflowSkipped;
        timRec->forcedTimerOverflowSkipped = 0;
        // accumulate total time timer is running
        timRec->runningTime += period + 1;
        timerOvrHandlerRec_t *cb = timRec->overflowCallbackActive;
        while(cb) {
            cb->fn(cb, period);
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


struct timerRec_all timerRecs;
// define and initialize timerDefs[] and timerRecPtrs
#include "timer_c_generated.inc"

#if 1
// handler for shared interrupts when both timers need to check status bits
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timIRQHandler(DEF_TIMER_TIM(i), DEF_TIMER_REC(i)); \
        timIRQHandler(DEF_TIMER_TIM(j), DEF_TIMER_REC(j)); \
        pinDbgLo(DBP_TIMER);                                            \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        pinDbgHi(DBP_TIMER);                                            \
        timIRQHandler(DEF_TIMER_TIM(i), DEF_TIMER_REC(i)); \
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

#if TIMER_USED_BITS & BIT(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
# if defined(STM32F10X)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);       // timer can't be shared
# endif
# if defined(STM32F303xC)
#  if TIMER_USED_BITS & BIT(16)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, 1, 16);  // both timers are in use
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 1);       // timer16 is not used
#  endif
# endif
#endif
#if TIMER_USED_BITS & BIT(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif
#if TIMER_USED_BITS & BIT(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif
#if TIMER_USED_BITS & BIT(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif
#if TIMER_USED_BITS & BIT(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
# if defined(STM32F10X_XL)
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);
# else  // f10x_hd, f30x
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
# endif
#endif
#if TIMER_USED_BITS & BIT(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if defined(STM32F303xC) && ((TIMER_USED_BITS & (BIT(1)|BIT(16))) == (BIT(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16);    // only timer16 is used, not timer1
#endif
#if TIMER_USED_BITS & BIT(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17);
#endif

void timerInit(void)
{

    for(unsigned i = 0; timerRecPtrs[i]; i++) {
        timerRec_t *timRec = timerRecPtrs[i];
        memset(timRec, 0, sizeof(*timRec));
        timRec->tim = timerDefs[i].tim;
        timRec->priority = 0xff;
        timRec->channels = timerDefs[i].channels;
        timRec->def = &timerDefs[i];
        for(unsigned j = 0; j < timRec->channels; j++) {
            timerChRec_t *timChRec = &timRec->channel[j];
            memset(timChRec, 0, sizeof(*timChRec));
            timChRec->timRec = timRec;
            timChRec->tim = timRec->tim;
            timChRec->channel = getTIM_Channel_x(j);
        }
    }

#ifdef TIME_USE_TIMER
    // initialize timer used for timing functions
    timerConfigure(DEF_TIMER_REC(TIME_TIMER), NVIC_PRIO_TIME_TIMER, 0, 1000000);
    timerCountRunningTime(DEF_TIMER_REC(TIME_TIMER), true, NVIC_PRIO_TIME_TIMER);
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
void timerForceOverflow(timerRec_t *timRec)
{
    TIM_TypeDef * tim = timRec->tim;

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        // Save the skipped count so that PPM reading will work on the same timer that was forced to overflow
        uint16_t cnt = tim->CNT;
        // restart and change forcedTimerOverflowSkipped only if previous overflow was already handled
        if(!(tim->SR & TIM_IT_Update)) {
            timRec->forcedTimerOverflowSkipped = tim->ARR - cnt;
            // Force an overflow by setting the UG bit
            tim->EGR |= TIM_EGR_UG;
        }
    }
}

void timerChForceOverflow(timerChRec_t *timChRec)
{
    timerForceOverflow(timChRec->timRec);
}

// using TIM to keep track of tiome (instead of systick)
#ifdef TIME_USE_TIMER

// retrun current time in microseconds
uint32_t micros(void)
{
    return timerGetRunningTimeCNTfast(DEF_TIMER_REC(TIME_TIMER));
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
        ATOMIC_BLOCK(NVIC_PRIO_TIME_TIMER) {   // make sure shared data are updated only once (millis is reentrant)
            if(microsLastCache == microsLast) {
                // ok, we won the right to update counter
                millisCounter += delta;
                microsLast += delta * 1000;
            } else {
                ; // Nothing to do - someone updated counter before we got here. Unless we spent whole millisecond since calculating delta, millisCounter must be correct
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
