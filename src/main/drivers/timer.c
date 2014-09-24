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

#include "gpio.h"
#include "system.h"
#include "callback.h"

#include "timer.h"

/* FreeFlight/Naze32 timer layout
    TIM2_CH1    RC1             PWM1
    TIM2_CH2    RC2             PWM2
    TIM2_CH3    RC3/UA2_TX      PWM3
    TIM2_CH4    RC4/UA2_RX      PWM4
    TIM3_CH1    RC5             PWM5
    TIM3_CH2    RC6             PWM6
    TIM3_CH3    RC7             PWM7
    TIM3_CH4    RC8             PWM8
    TIM1_CH1    PWM1            PWM9
    TIM1_CH4    PWM2            PWM10
    TIM4_CH1    PWM3            PWM11
    TIM4_CH2    PWM4            PWM12
    TIM4_CH3    PWM5            PWM13
    TIM4_CH4    PWM6            PWM14

    RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    RX2  TIM2_CH2 PA1
    RX3  TIM2_CH3 PA2 [also UART2_TX]
    RX4  TIM2_CH4 PA3 [also UART2_RX]
    RX5  TIM3_CH1 PA6 [also ADC_IN6]
    RX6  TIM3_CH2 PA7 [also ADC_IN7]
    RX7  TIM3_CH3 PB0 [also ADC_IN8]
    RX8  TIM3_CH4 PB1 [also ADC_IN9]

    Outputs
    PWM1 TIM1_CH1 PA8
    PWM2 TIM1_CH4 PA11
    PWM3 TIM4_CH1 PB6 [also I2C1_SCL]
    PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    PWM5 TIM4_CH3 PB8
    PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels
*/

/* some magic to use preprocessor for timer mapping */
#define TIM_N(i) (1<<(i))

#if (defined(STM32F10X) || defined(NAZE)) && !defined(CC3D)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD},          // PWM1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD},          // PWM2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_IPD},          // PWM3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_IPD},          // PWM4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, Mode_IPD},          // PWM5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD},          // PWM6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD},          // PWM7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD},          // PWM8
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD},       // PWM9
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD},      // PWM10
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD},          // PWM11
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, Mode_IPD},          // PWM12
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, Mode_IPD},          // PWM13
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, Mode_IPD}           // PWM14
};

const timerHardware_t timerQueueHardware = 
{ TIM1, GPIOA, Pin_9, TIM_Channel_2, TIM1_CC_IRQn, 0, Mode_IPD};


#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#endif

#if defined(CC3D)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD}, // S1_IN
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD}, // S2_IN - GPIO_PartialRemap_TIM3
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD}, // S3_IN
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD}, // S4_IN
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD}, // S5_IN
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD}, // S6_IN

    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S1_OUT
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S2_OUT
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    // S3_OUT
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, GPIO_Mode_AF_PP}, // S4_OUT
    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF_PP},    // S5_OUT - GPIO_PartialRemap_TIM3
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF_PP},    // S6_OUT
};

#define USED_TIMERS  TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#endif

#if (defined(STM32F303) || defined(STM32F3DISCOVERY)) && !(defined(CHEBUZZF3) || defined(NAZE32PRO))
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP_PD},             // PWM1 - PA8
    { TIM16, GPIOB, Pin_8, TIM_Channel_1, TIM1_UP_TIM16_IRQn, 0, Mode_AF_PP_PD},      // PWM2 - PB8
    { TIM17, GPIOB, Pin_9, TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD}, // PWM3 - PB9
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},             // PWM4 - PC6
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},             // PWM5 - PC7
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},             // PWM6 - PC8
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_AF_PP_PD},                // PWM7 - PB1
    { TIM3, GPIOA, Pin_4, TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP_PD},                // PWM8 - PA2
    { TIM4, GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM9 - PD12
    { TIM4, GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM10 - PD13
    { TIM4, GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM11 - PD14
    { TIM4, GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM12 - PD15
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP},                   // PWM13 - PA1
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP}                    // PWM14 - PA2
};

#define USED_TIMERS  TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(16)|TIM_N(17)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD)

#endif

#if defined(CHEBUZZF3)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // INPUTS CH1-8
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP_PD},            // PWM1 - PA8
    { TIM16, GPIOB, Pin_8,  TIM_Channel_1, TIM1_UP_TIM16_IRQn, 0, Mode_AF_PP_PD},      // PWM2 - PB8
    { TIM17, GPIOB, Pin_9,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD}, // PWM3 - PB9
    { TIM8,  GPIOC, Pin_6,  TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},            // PWM4 - PC6
    { TIM8,  GPIOC, Pin_7,  TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},            // PWM5 - PC7
    { TIM8,  GPIOC, Pin_8,  TIM_Channel_3, TIM8_CC_IRQn, 1, Mode_AF_PP_PD},            // PWM6 - PC8
    { TIM15, GPIOF, Pin_9,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 0, Mode_AF_PP_PD},     // PWM7 - PF9
    { TIM15, GPIOF, Pin_10, TIM_Channel_2, TIM1_BRK_TIM15_IRQn, 0, Mode_AF_PP_PD},     // PWM8 - PF10

    // OUTPUTS CH1-10
    { TIM4,  GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM9 - PD12
    { TIM4,  GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM10 - PD13
    { TIM4,  GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM11 - PD14
    { TIM4,  GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn, 0, Mode_AF_PP},                  // PWM12 - PD15
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP},                  // PWM13 - PA1
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP},                  // PWM14 - PA2
    { TIM2,  GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP},                  // PWM15 - PA3
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn, 0, Mode_AF_PP},                  // PWM16 - PB0
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn, 0, Mode_AF_PP},                  // PWM17 - PB1
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP}                   // PWM18 - PA4
};

#define USED_TIMERS  TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(15)|TIM_N(16)|TIM_N(17)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF)

#endif

#ifdef NAZE32PRO
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 0, Mode_AF_PP_PD}, // PA8 - AF6
    { TIM1,  GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn, 0, Mode_AF_PP_PD}, // PA9 - AF6
    { TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 0, Mode_AF_PP_PD}, // PA10 - AF6
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,    0, Mode_AF_PP_PD}, // PB3 - AF2
    { TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,    0, Mode_AF_PP_PD}, // PB6 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,    0, Mode_AF_PP_PD}, // PB7 - AF2 - not working yet
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,    0, Mode_AF_PP_PD}, // PB8 - AF2
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,    0, Mode_AF_PP_PD}, // PB9 - AF2

    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP}, // PA0 - untested
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP}, // PA1 - untested
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP}, // PA2 - untested
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP}, // PA3 - untested
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP}, // PA6 - untested
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP}, // PA7 - untested
};

#define USED_TIMERS  TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(15)|TIM_N(16)|TIM_N(17)


#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#define MAX_TIMERS BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4


// type sizes for different CPU versions
#if 0  // TODO - in timer.h
#if defined(STM32F303xC)
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
#elif defined(STM32F10X_MD)
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
#else 
# error "Unknown CPU defined"
#endif
#endif

#define TIMER_IRQ_PRIORITY 0
#define TIMER_IRQ_SUBPRIORITY 1


#define TIM_IT_CCx(ch) (TIM_IT_CC1<<((ch)/4))

typedef struct timerConfig_s {
    timerCCCallbackPtr *edgeCallback;
    timerCCCallbackPtr *overflowCallback;
    void* data;
} timerConfig_t;

timerConfig_t timerConfig[MAX_TIMERS][CC_CHANNELS_PER_TIMER];


#define TIMER_INDEX(i) BITCOUNT((TIM_N(i)-1)&USED_TIMERS) 

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_(tim, index) case ((unsigned)tim): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))

// let gcc do the work, switch should be quite optimized
    switch((unsigned)tim) {
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
   
}

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel>>2;
}

void configureTimerChannelCallbacks(TIM_TypeDef *tim, uint8_t channel, void* data, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback)
{
    assert_param(IS_TIM_CHANNEL(channel));

    uint8_t timerIndex = lookupTimerIndex(tim);
    
    if (timerIndex >= MAX_TIMERS) {
        return;
    }
    uint8_t channelIndex=lookupChannelIndex(channel);
    timerConfig[timerIndex][channelIndex].edgeCallback = edgeCallback;
    timerConfig[timerIndex][channelIndex].overflowCallback = overflowCallback;
    timerConfig[timerIndex][channelIndex].data = data;
}

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel)
{
    switch (channel) {
        case TIM_Channel_1:
            TIM_ITConfig(tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback)
{
//    configureTimerChannelCallbacks(timerHardwarePtr->tim, timerHardwarePtr->channel, (void*)(int)reference, edgeCallback, overflowCallback);
//    configureTimerInputCaptureCompareChannel(timerHardwarePtr->tim, timerHardwarePtr->channel);
//    if (overflowCallback) {
//        TIM_ITConfig(timerHardwarePtr->tim, TIM_IT_Update, ENABLE);
//    }
}

void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMER_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMER_IRQ_SUBPRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
    timerNVICConfigure(timerHardwarePtr->irq);
}

void timerConfigHandled(const timerHardware_t *timHw)
{
    timerConfigure(timHw, 0, 1);
}


timerConfig_t *findTimerConfig(TIM_TypeDef *tim, uint16_t channel)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if(timerIndex >= MAX_TIMERS) 
        return NULL;   // maybe we shuld return some dummy value
    return &timerConfig[timerIndex][lookupChannelIndex(channel)];
}

void timerChCfgCallbacks(const timerHardware_t* timHw, void* data, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= MAX_TIMERS) {
        return;
    }
    uint8_t channelIndex=lookupChannelIndex(timHw->channel);
    if(edgeCallback==NULL)   // disable irq before changing setting callback to NULL 
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
    // setup callback info
    timerConfig[timerIndex][channelIndex].edgeCallback = edgeCallback;
    timerConfig[timerIndex][channelIndex].overflowCallback = overflowCallback;    
    timerConfig[timerIndex][channelIndex].data = data;
    // enable channel (and possibly timer) IRQ
    if(edgeCallback)
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);
    if(overflowCallback)       // todo - allow disabling overflow callbach when no channel uses it
        TIM_ITConfig(timHw->tim, TIM_IT_Update, ENABLE);
}

void timerChCfgGPIO(const timerHardware_t* timHw, GPIO_Mode mode)
{
    gpio_config_t cfg;
    
    cfg.pin = timHw->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(timHw->gpio, &cfg);
}


void timerChConfigIC(const timerHardware_t* timHw, bool polarityRising)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising?TIM_ICPolarity_Rising:TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

void timerChICPolarity(const timerHardware_t* timHw, bool polarityRising)
{
    timCCER_t tmpccer=timHw->tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P<<timHw->channel);
    tmpccer |= polarityRising?(TIM_ICPolarity_Rising << timHw->channel):(TIM_ICPolarity_Falling << timHw->channel);
    timHw->tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCR(const timerHardware_t* timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1+timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool outInvert)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    if(outEnable) {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity = outInvert?TIM_OCPolarity_Low:TIM_OCPolarity_High;
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



static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t timerConfig[])
{
    uint16_t capture;
    uint8_t channelIndex;
    unsigned tim_status;
    tim_status=tim->SR & tim->DIER;
#if 1
    switch(__builtin_clz(tim_status)) {
    case __builtin_clz(TIM_IT_Update):
        tim->SR = ~TIM_IT_Update;
        capture = tim->ARR;
        
        for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++)
            if (timerConfig[channelIndex].overflowCallback) 
                timerConfig[channelIndex].overflowCallback(timerConfig[channelIndex].data, capture);
        break;
    case __builtin_clz(TIM_IT_CC1):
        tim->SR = ~TIM_IT_CC1;
        timerConfig[0].edgeCallback(timerConfig[0].data, tim->CCR1);
        break;
    case __builtin_clz(TIM_IT_CC2):        
        tim->SR = ~TIM_IT_CC2;
        timerConfig[1].edgeCallback(timerConfig[1].data, tim->CCR2);
        break;
    case __builtin_clz(TIM_IT_CC3):
        tim->SR = ~TIM_IT_CC3;
        timerConfig[2].edgeCallback(timerConfig[2].data, tim->CCR3);
        break;
    case __builtin_clz(TIM_IT_CC4):
        tim->SR = ~TIM_IT_CC4;
        timerConfig[3].edgeCallback(timerConfig[3].data, tim->CCR4);
        break;
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim->SR = ~TIM_IT_Update;
        capture = tim->ARR;

        for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++)
            if (timerConfig[channelIndex].overflowCallback) 
                timerConfig[channelIndex].overflowCallback(timerConfig[channelIndex].data, capture);
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim->SR = ~TIM_IT_CC1;
        timerConfig[0].edgeCallback(timerConfig[0].data, tim->CCR1);
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim->SR = ~TIM_IT_CC2;
        timerConfig[1].edgeCallback(timerConfig[1].data, tim->CCR2);
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim->SR = ~TIM_IT_CC3;
        timerConfig[2].edgeCallback(timerConfig[2].data, tim->CCR3);
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim->SR = ~TIM_IT_CC4;
        timerConfig[3].edgeCallback(timerConfig[3].data, tim->CCR4);
    }
#endif
}
# if 0
#define _TIM_IRQ_HANDLER(name, i)                                      \
    void name(void)                                                    \
    {                                                                  \
        timCCxHandler(TIM ## i, timerConfig[TIMER_INDEX(i)]);          \
    }                                                         
# elseif 0
#define _TIM_IRQ_HANDLER(name, i)                                      \
    uint32_t dbghist_##name[32];                                       \
    void name(void)                                                    \
    {                                                                  \
        uint16_t start=TIM1->CNT;                                      \
        timCCxHandler(TIM ## i, timerConfig[TIMER_INDEX(i)]);          \
        start=TIM1->CNT-start;                                         \
        if(start>31) start=31;                                         \
        dbghist_##name[start]++;                                       \
    }     
# else                                                    
#define _TIM_IRQ_HANDLER(name, i)                                      \
    void name(void)                                                    \
    {                                                                  \
        digitalHi(GPIOB, Pin_9);                                       \
        timCCxHandler(TIM ## i, timerConfig[TIMER_INDEX(i)]);          \
        digitalLo(GPIOB, Pin_9);                                       \
    }                                                         
# endif
#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1)
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2)
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3)
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4)
#endif
#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8)
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15)
#endif
#if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16)
#endif
#if USED_TIMERS & TIM_N(17)
_TIM_IRQ_HANDLER(TIM1_TRG_COM_TIM17_IRQHandler, 17)
#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

// TODO - DEBUG
    gpio_config_t cfg;
    
    cfg.pin = Pin_8|Pin_9;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
    gpioInit(GPIOB, &cfg);

#ifdef CC3D
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif

#ifdef TIMER_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_AHB_PERIPHERALS
    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
#endif


#ifdef STM32F303
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_6);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,  GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_4);
#ifdef CHEBUZZF3
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource9,  GPIO_AF_3);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_3);
#endif
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_2);
#endif

#ifdef NAZE32PRO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_6);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,  GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,  GPIO_AF_2);
#endif
}
