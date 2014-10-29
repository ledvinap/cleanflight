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

#include "nvic.h"

#include "gpio.h"
#include "system.h"
#include "callback.h"

#include "timer.h"
#include "timer_impl.h"

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4              // TIM_Channel_1..4

#define TIM_IT_CCx(ch) (TIM_IT_CC1<<((ch)/4))

typedef struct timerConfig_s {
    timerCCHandlerRec_t* edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t* overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t* overflowCallbackActive; // null-terminated linkded list of active overflow callbacks    
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[USABLE_TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

// retunt index of timer in timer table. Lowest timer has index 0
#define TIMER_INDEX(i) BITCOUNT((TIM_N(i)-1)&USED_TIMERS)

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
    default:  return -1;  // make sure final index is out of range
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

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel>>2;
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;


    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}



void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    configTimeBase(timerHardwarePtr->tim, 0, mhz);
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);
}

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority)
{
    unsigned channel=timHw-timerHardware;
    if(channel>=USABLE_TIMER_CHANNEL_COUNT)
        return;
    
    timerChannelInfo[channel].type=type;
    unsigned timer=lookupTimerIndex(timHw->tim);
    if(timer>=USED_TIMER_COUNT) 
        return;
    if(irqPriority<timerInfo[timer].priority) {
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);

        NVIC_InitTypeDef NVIC_InitStructure;
        
        NVIC_InitStructure.NVIC_IRQChannel = timHw->irq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure); 
        
        timerInfo[timer].priority=irqPriority;
    }
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn=fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn=fn;
    self->next=NULL;
}

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef* tim) {
    timerOvrHandlerRec_t **chain=&cfg->overflowCallbackActive;
    register uint8_t saved_basepri= __get_BASEPRI();
    __set_BASEPRI(NVIC_PRIO_TIMER); asm volatile ("" ::: "memory"); 
    for(int i=0;i<CC_CHANNELS_PER_TIMER;i++)
        if(cfg->overflowCallback[i]) {
            *chain=cfg->overflowCallback[i];
            chain=&cfg->overflowCallback[i]->next;
        }
    *chain=NULL;
    __set_BASEPRI(saved_basepri);
    // enable or disable IRQ
    TIM_ITConfig(tim, TIM_IT_Update, cfg->overflowCallbackActive?ENABLE:DISABLE);
}

void timerChConfigCallbacks(const timerHardware_t* timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex=lookupChannelIndex(timHw->channel);
    if(edgeCallback==NULL)   // disable irq before changing callback to NULL 
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;    
    // enable channel IRQ
    if(edgeCallback)
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);
    
    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}

void timerChConfigCallbacksDual(const timerHardware_t* timHw, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint16_t chLo=timHw->channel&~TIM_Channel_2;   // lower channel
    uint16_t chHi=timHw->channel|TIM_Channel_2;    // upper channel 
    uint8_t channelIndex=lookupChannelIndex(chLo);   // get index of lower channel

    if(edgeCallbackLo==NULL)   // disable irq before changing setting callback to NULL 
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chLo), DISABLE);
    if(edgeCallbackHi==NULL)   // disable irq before changing setting callback to NULL 
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chHi), DISABLE);
    
    // setup callback info
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex+1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;    
    timerConfig[timerIndex].overflowCallback[channelIndex+1] = NULL;    

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

void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState) {
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
}

void timerChITConfig(const timerHardware_t* timHw, FunctionalState newState)
{
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), newState);
}

void timerChClearCCFlag(const timerHardware_t* timHw)
{
    TIM_ClearFlag(timHw->tim, TIM_IT_CCx(timHw->channel));
}

void timerChConfigGPIO(const timerHardware_t* timHw, GPIO_Mode mode)
{
    gpio_config_t cfg;
    
    cfg.pin = timHw->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(timHw->gpio, &cfg);
}


void timerChConfigIC(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising?TIM_ICPolarity_Rising:TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0f;

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

// configure dual channel channel for capture
// polarity is for first capture
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    // configure direct channel
    TIM_ICStructInit(&TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising?TIM_ICPolarity_Rising:TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0f;
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel = timHw->channel ^ TIM_Channel_2;   // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity = directRising?TIM_ICPolarity_Falling:TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}



void timerChICPolarity(const timerHardware_t* timHw, bool polarityRising)
{
    timCCER_t tmpccer=timHw->tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P<<timHw->channel);
    tmpccer |= polarityRising?(TIM_ICPolarity_Rising << timHw->channel):(TIM_ICPolarity_Falling << timHw->channel);
    timHw->tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1+(timHw->channel|TIM_Channel_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1+(timHw->channel&~TIM_Channel_2));
}



volatile timCCR_t* timerChCCR(const timerHardware_t* timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1+timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    if(outEnable) {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;        
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = stateHigh?TIM_OCPolarity_High:TIM_OCPolarity_Low;
    } else {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;        
    }
    
    switch (timHw->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    }
}



static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t* timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status=tim->SR & tim->DIER;
#if 1
    while(tim_status) {                 // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        unsigned bit=__builtin_clz(tim_status);  
        unsigned mask=~(0x80000000>>bit);
        tim->SR = mask;
        tim_status &= mask;
        switch(bit) {
        case __builtin_clz(TIM_IT_Update):
            capture = tim->ARR;
            timerOvrHandlerRec_t *cb=timerConfig->overflowCallbackActive;
            while(cb) {
                cb->fn(cb, capture);
                cb=cb->next;
            }
            break;
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
        capture = tim->ARR;
        timerOvrHandlerRec_t *cb=timerConfig->overflowCallbackActive;
        while(cb) {
            cb->fn(cb, capture);
            cb++;
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

#define _TIM_IRQ_HANDLER2(name, i, j)                                    \
    void name(void)                                                    \
    {                                                                  \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TIM ## j, &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy                                                    
# if 0
#define _TIM_IRQ_HANDLER(name, i)                                      \
    void name(void)                                                    \
    {                                                                  \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy                                                         
# elseif 0
#define _TIM_IRQ_HANDLER(name, i)                                      \
    uint32_t dbghist_##name[32];                                       \
    void name(void)                                                    \
    {                                                                  \
        uint16_t start=TIM1->CNT;                                      \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        start=TIM1->CNT-start;                                         \
        if(start>31) start=31;                                         \
        dbghist_##name[start]++;                                       \
    } struct dummy    
# else                                                    
#define _TIM_IRQ_HANDLER(name, i)                                      \
    void name(void)                                                    \
    {                                                                  \
        digitalHi(GPIOB, Pin_9);                                       \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);         \
        digitalLo(GPIOB, Pin_9);                                       \
    }  struct dummy                                                       
# endif

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
# if defined(STM32F10X)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);
# endif
# ifdef STM32F303xC
#  if USED_TIMERS & TIM_N(16);
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, 1, 16);
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 1);
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
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if defined(STM32F303xC) && ((USED_TIMERS & (TIM_N(1)|TIM_N(16))) == (TIM_N(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16)
#endif

#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17)
#endif



void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

// TODO - DEBUG
    {
        gpio_config_t cfg;
    
        cfg.pin = Pin_9;
        cfg.mode = Mode_Out_PP;
        cfg.speed = Speed_10MHz;
        gpioInit(GPIOB, &cfg);
    }
    // call target-specific initialization routine
    timerInitTarget();
#ifdef CC3D
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif


#ifdef STM32F303xC
    for (uint8_t timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        GPIO_PinAFConfig(timerHardwarePtr->gpio, (uint16_t)timerHardwarePtr->gpioPinSource, timerHardwarePtr->alternateFunction);
    }
#endif

#if 0
#if defined(STMF3DISCOVERY) || defined(NAZE32PRO)
    // FIXME move these where they are needed.
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
#endif
#endif

// initialize timer channel structures
    for(int i=0;i<USABLE_TIMER_CHANNEL_COUNT;i++) {
        timerChannelInfo[i].type=TYPE_FREE;
    }
    for(int i=0;i<USED_TIMER_COUNT;i++) {
        timerInfo[i].priority=~0;
    }

}

// finish configuring timers after allocation phase
// start timers
void timerStart(void)
{
#if 0
    for(unsigned timer=0;timer<USED_TIMER_COUNT;timer++) {
        int priority=-1;
        int irq=-1;
        for(unsigned hwc=0;hwc<USABLE_TIMER_CHANNEL_COUNT;hwc++)
            if(timerChannelInfo[hwc].type!=TYPE_FREE && timerHardware[hwc].tim==usedTimers[timer]) {
                // TODO - move IRQ to timer info
                irq=timerHardware[hwc].irq;
                
            }
        // TODO - aggregate required timer paramaters
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);
        if(priority>=0) {  // maybe none of the channels was configured
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
