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
// target timer usage configuration
#include "target_timer.h"
#include "timer_def_generated.h"

#define DEF_TIMER_TIM(tim) CONCAT(TIM, tim)
#define DEF_TIMER_REC(tim) (CONCAT(&timerRecs.rec_TIM, tim))

// initializer for timerDef_t record
#if defined(STM32F10X)
# define DEF_TIMCH(pinid_, tim_, tim_ch_) {                             \
        .timerIdx = (tim_),                                             \
        .channelIdx = (tim_ch_),                                        \
        .ioTag = DEFIO_TAG(pinid_),                                     \
    }                                                                   \
    /**/
#elif defined(STM32F303xC)
# define DEF_TIMCH(pinid, tim_, tim_ch_, pin_af_) {                     \
        .timerIdx = (tim_),                                             \
        .channelIdx = (tim_ch_),                                        \
        .ioTag = DEFIO_TAG(pinid),                                      \
        .pinAf = pin_af_                                                \
    }                                                                   \
    /**/
#endif

// define timer parameteters according to selected CPU
// defined for all available timers, target_timer.h select actually used timers
// each tuple is (CC_IRQn, UP_IRQn, outputEnable, RRC)
#if defined(STM32F10X)
# define DEF_TIMER_INFO_TIM1  (TIM1_CC_IRQn, TIM1_UP_IRQn,       true,  RCC_APB2(TIM1));
# define DEF_TIMER_INFO_TIM2  (TIM2_IRQn,    TIM2_IRQn,          false, RCC_APB1(TIM2));
# define DEF_TIMER_INFO_TIM3  (TIM3_IRQn,    TIM3_IRQn,          false, RCC_APB1(TIM2));
# define DEF_TIMER_INFO_TIM4  (TIM4_IRQn,    TIM4_IRQn,          false, RCC_APB1(TIM4));
// TODO - TIM6, TIM7
# if defined(STM32F10X_XL)
#  define DEF_TIMER_INFO_TIM8 (TIM8_CC_IRQn, TIM8_UP_TIM13_IRQn, true,  RCC_APB2(TIM8));
# elif defined( STM32F10X_HD)
#  define DEF_TIMER_INFO_TIM8 (TIM8_CC_IRQn, TIM8_UP_IRQn,       true,  RCC_APB2(TIM8));
# endif
#elif defined(STM32F303xC)
# define DEF_TIMER_INFO_TIM1  (TIM1_CC_IRQn,             TIM1_UP_TIM16_IRQn,      true,  RCC_APB2(TIM1))
# define DEF_TIMER_INFO_TIM2  (TIM2_IRQn,                TIM2_IRQn,               false, RCC_APB1(TIM2))
# define DEF_TIMER_INFO_TIM3  (TIM3_IRQn,                TIM3_IRQn,               false, RCC_APB1(TIM3))
# define DEF_TIMER_INFO_TIM4  (TIM4_IRQn,                TIM4_IRQn,               false, RCC_APB1(TIM4))
# define DEF_TIMER_INFO_TIM6  (TIM6_DAC_IRQn,            TIM6_DAC_IRQn,           false, RCC_APB1(TIM6))
# define DEF_TIMER_INFO_TIM7  (TIM7_IRQn,                TIM7_IRQn,               false, RCC_APB1(TIM7))
# define DEF_TIMER_INFO_TIM8  (TIM8_CC_IRQn,             TIM8_UP_IRQn,            true,  RCC_APB2(TIM8))
# define DEF_TIMER_INFO_TIM15 (TIM1_BRK_TIM15_IRQn,     TIM1_BRK_TIM15_IRQn,     true,  RCC_APB2(TIM15))
# define DEF_TIMER_INFO_TIM16 (TIM1_UP_TIM16_IRQn,      TIM1_UP_TIM16_IRQn,      true,  RCC_APB2(TIM16))
# define DEF_TIMER_INFO_TIM17 (TIM1_TRG_COM_TIM17_IRQn, TIM1_TRG_COM_TIM17_IRQn, true,  RCC_APB2(TIM17))
#else
# error "TIMER_INFO is not defined for current cpu"
#endif


// emit TIMER definition record
// TODO - user timer type instead of outputs enable
#define DEF_TIMER_DEFINE_II(tim_,channels_, iCC_, iUP_, outEna_, rcc_) { \
        .tim = DEF_TIMER_TIM(tim_),                                   \
        .irqCC = iCC_,                                                  \
        .irqUP = iUP_,                                                  \
        .channels = channels_,                                          \
        .outputsNeedEnable = outEna_,                                   \
        .rcc = rcc_,                                                    \
    }                                                                   \
    /**/
#define DEF_TIMER_DEFINE_EXPAND(CC_IRQn, UP_IRQn, outputEnable, RCC) CC_IRQn, UP_IRQn, outputEnable, RCC
#define DEF_TIMER_DEFINE_I(tim, channels, info) DEF_TIMER_DEFINE_II(tim, channels, info)
#define DEF_TIMER_DEFINE(tim)                                           \
    DEF_TIMER_DEFINE_I(tim,                                             \
                       CONCAT(TARGET_TIMER_TIM, tim),                   \
                       EXPAND(DEF_TIMER_DEFINE_EXPAND CONCAT(DEF_TIMER_INFO_TIM, tim))) \
    /**/
