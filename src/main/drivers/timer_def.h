#pragma once

#include "common/utils.h"
#include <boost/preprocessor/repetition/enum.hpp>
#include <boost/preprocessor/facilities/apply.hpp>

#include "target_timer.h"

// tim, ch are numbers (TIM<tim>, TIM_Channel_<ch>)

#define DEFIO_TIMERCH_REC(tim, ch) (&(DEFIO_TIMER_REC(tim)->channel[(ch)-1]))
#define DEFIO_TIMER_REC_NAME(tim) CONCAT(TIMER_TIM, tim)
#define DEFIO_TIMER_REC(tim) (&DEFIO_TIMER_REC_NAME(tim))

#define DEFIO_TIMER_DEF_NAME(tim) CONCAT(timerDef_TIM, tim)
#define DEFIO_TIMER_DEF(tim) (&DEFIO_TIMER_DEF_NAME(tim))

#define DEFIO_TIMER_TIM(tim) CONCAT(TIM, tim)
#define DEFIO_TIMER_TIM_CHANNEL(ch) CONCAT(TIM_Channel_, ch)

#if defined(STM32F10X)
# define DEF_TIMCH(pinid_, tim_, tim_ch_) {                             \
        .timerIdx = (tim_),                                             \
        .channelIdx = (tim_ch_),                                        \
        .ioTag = DEFIO_TAG(pinid_),                                     \
    }                                                                   \
                                          /**/
#elif defined(STM32F303xC)
# define DEF_TIMCH(pinid, tim_, tim_ch_, pin_af_) {                     \
        .timerIdx = (tim_),                           \
        .channelIdx = (tim_ch_),                    \
        .ioTag = DEFIO_TAG(pinid),                                      \
        .pinAf = pin_af_                                                \
        }                                                               \
        /**/
#endif

#define ENUM_DATA(z,n,data) ENUM_DATA_I data
#define ENUM_DATA_I(...) __VA_ARGS__

// emit actual TIMER definitions
// TODO - user timer type instead of outputs enable
#define DEF_TIMER_DEFINE(tim_, iCC, iUP, outEna, rcc_, channels_)        \
    timerRec_t DEFIO_TIMER_REC_NAME(tim_)                               \
        __attribute__ ((section (".tab.timer.rec." BOOST_PP_IF(BOOST_PP_LESS(tim_, 10), "0", "") STR(tim_) ))) = \
        DEFIO_TIMER_REC_INITIALIZER;                                    \
    timerChRec_t BOOST_PP_CAT(DEFIO_TIMER_REC_NAME(tim_), _ch)[channels_] \
        __attribute__ ((section (".tab.timer.rec." BOOST_PP_IF(BOOST_PP_LESS(tim_, 10), "0", "") STR(tim_) ))) = \
    {BOOST_PP_ENUM(channels_, ENUM_DATA, (DEFIO_TIMERCH_REC_INITIALIZER))}; \
    const timerDef_t DEFIO_TIMER_DEF_NAME(tim_)                         \
        __attribute__ ((section (".tab.timer.def." BOOST_PP_IF(BOOST_PP_LESS(tim_, 10), "0", "") STR(tim_) ))) = { \
        .tim = DEFIO_TIMER_TIM(tim_),                                         \
        .irqCC = iCC,                                                   \
        .irqUP = iUP,                                                   \
        .channels = channels_,                                           \
        .outputsNeedEnable = outEna,                                    \
        .rcc = rcc_,                                                    \
    }                                                                   \
    /**/

#define DEF_TIMER(tim_, iCC, iUP, outEna, rcc_)                         \
    extern timerRec_t DEFIO_TIMER_REC_NAME(tim_)      \
    /**/
