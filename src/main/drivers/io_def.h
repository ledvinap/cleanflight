#pragma once

#include "common/utils.h"

// define macros for easy specification of IO pins.

// Structure defining available IO pins is defined using DEF_IO macro. Parameters to this macro are intended to be easily readable.
// The paramaters are concatenated before expansion, so no namespace collision should occut (we can use TIMx, etc. as parameter)
// this also allows using macro parameters in differenct context. This mechanism may be improved if boost preprocessor (BOOST PP) is 
// used in future.

// GPIOx
#define DEFIO_GPIO_ID__GPIOA 1
#define DEFIO_GPIO_ID__GPIOB 2
#define DEFIO_GPIO_ID__GPIOC 3
#define DEFIO_GPIO_ID__GPIOF 4

#define DEFIO_GPIO_LETTER__1 A
#define DEFIO_GPIO_LETTER__2 B
#define DEFIO_GPIO_LETTER__3 C
#define DEFIO_GPIO_LETTER__4 F


#define DEFIO_GPIO_LETTER(gpio) CONCAT(DEFIO_GPIO_LETTER__, gpio)

// PINx
#define DEFIO_PIN_ID__PIN0 0
#define DEFIO_PIN_ID__PIN1 1
#define DEFIO_PIN_ID__PIN2 2
#define DEFIO_PIN_ID__PIN3 3
#define DEFIO_PIN_ID__PIN4 4
#define DEFIO_PIN_ID__PIN5 5
#define DEFIO_PIN_ID__PIN6 6
#define DEFIO_PIN_ID__PIN7 7
#define DEFIO_PIN_ID__PIN8 8
#define DEFIO_PIN_ID__PIN9 9
#define DEFIO_PIN_ID__PIN10 10
#define DEFIO_PIN_ID__PIN11 11
#define DEFIO_PIN_ID__PIN12 12
#define DEFIO_PIN_ID__PIN13 13
#define DEFIO_PIN_ID__PIN14 14
#define DEFIO_PIN_ID__PIN15 15

// TIMx
#define DEFIO_TIM_ID__TIM1 1
#define DEFIO_TIM_ID__TIM2 2
#define DEFIO_TIM_ID__TIM3 3
#define DEFIO_TIM_ID__TIM4 4
#define DEFIO_TIM_ID__TIM6 6
#define DEFIO_TIM_ID__TIM7 7
#define DEFIO_TIM_ID__TIM8 8
#define DEFIO_TIM_ID__TIM15 15
#define DEFIO_TIM_ID__TIM16 16
#define DEFIO_TIM_ID__TIM17 17

// TIMCH to TIM_Channel_x mapping
#define DEFIO_TIMCH_ID__NA 0
#define DEFIO_TIMCH_ID__TIMCH1 1
#define DEFIO_TIMCH_ID__TIMCH2 2
#define DEFIO_TIMCH_ID__TIMCH3 3
#define DEFIO_TIMCH_ID__TIMCH4 4

#define DEFIO_TIMERCH_REC(tim, ch) (DEFIO_TIMER_REC(tim).channel[(ch)-1])
#define DEFIO_TIMER_REC(tim) CONCAT(timerRec_TIM, tim)
#define DEFIO_TIMER_DEF(tim) CONCAT(TIMER_TIM, tim)
#define DEFIO_IO_DEF(gpio, pin) CONCAT(IO_P, CONCAT(DEFIO_GPIO_LETTER(gpio), pin))
#define DEFIO_IO_REC(gpio, pin) CONCAT(ioRec_P, CONCAT(DEFIO_GPIO_LETTER(gpio), pin))
#define DEFIO_TIM(tim) CONCAT(TIM, tim)
#define DEFIO_GPIO(gpio) CONCAT(GPIO, DEFIO_GPIO_LETTER(gpio))
#define DEFIO_PIN(pin) CONCAT(Pin_, pin)
#define DEFIO_TIM_CHANNEL(ch) CONCAT(TIM_Channel_, ch)

// the ## operator must be used directly here - the parameter should not be expanded
// some magic may be used if expansion is desirable (#define DEFIO_GPIO__EXPAND(x) CONCAT(GPIO, x) )
#define DEF_TIMCH(gpio_, pin_, tim_, tim_ch_) {                             \
        .rec = &DEFIO_TIMERCH_REC(DEFIO_TIM_ID__  ## tim_, DEFIO_TIMCH_ID__ ## tim_ch_), \
            .timerDef = &DEFIO_TIMER_DEF(DEFIO_TIM_ID__ ## tim_),       \
            .ioDef = &DEFIO_IO_DEF(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_), \
            .tim = DEFIO_TIM(DEFIO_TIM_ID__ ## tim_),                   \
            .gpio = DEFIO_GPIO(DEFIO_GPIO_ID__ ## gpio_),               \
            .pin = DEFIO_PIN(DEFIO_PIN_ID__ ## pin_),                   \
            .channel = DEFIO_TIM_CHANNEL(DEFIO_TIMCH_ID__ ## tim_ch_)   \
}                                                                       \
/**/

#if defined(IO_DEF_DEFINE)


// we are included in C file, emit actual IO definitions
#define DEF_IO(gpio_, pin_)                                             \
    struct ioRec_s DEFIO_IO_REC(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_); \
    const struct ioDef_s DEFIO_IO_DEF(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_) \
        __attribute__ ((section (".text.gpio." STR(DEFIO_IO_DEF(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_)) ))) \
        = {                                                             \
        .gpio = DEFIO_GPIO(DEFIO_GPIO_ID__ ## gpio_),                   \
        .pin = DEFIO_PIN(DEFIO_PIN_ID__ ## pin_),                       \
        .rec = &DEFIO_IO_REC(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_) \
    }                                                                   \
    /**/
#else
struct ioDef_s;
#define DEF_IO(gpio_, pin_) \
    extern const struct ioDef_s DEFIO_IO_DEF(DEFIO_GPIO_ID__ ## gpio_, DEFIO_PIN_ID__ ## pin_) \
    /**/
#endif

#if defined(IO_DEF_DEFINE)
// we are included in C file, emit actual TIMER definitions
// TODO - user timer type instead of outputs enable
#define DEF_TIMER(tim_, iCC, iUP, outEna, rcc_)                              \
    struct timerRec_s DEFIO_TIMER_REC(DEFIO_TIM_ID__ ## tim_);          \
    const struct timerDef_s DEFIO_TIMER_DEF(DEFIO_TIM_ID__ ## tim_)     \
        __attribute__ ((section (".text.timer." STR(DEFIO_TIMER_DEF(DEFIO_TIM_ID__ ## tim_)) ))) \
        = {                                                             \
        .rec = &DEFIO_TIMER_REC(DEFIO_TIM_ID__ ## tim_),                \
        .tim = DEFIO_TIM(DEFIO_TIM_ID__ ## tim_),                       \
        .irqCC = iCC,                                                   \
        .irqUP = iUP,                                                   \
        .channels = CC_CHANNELS_PER_TIMER,                              \
        .outputsNeedEnable = outEna,                                    \
        .rcc = rcc_,                                                    \
    }                                                                   \
    /**/
#else
struct ioDef_s;
struct timerRec_s;
#define DEF_TIMER(tim_, iCC, iUP, outEna, rcc_)                         \
    extern const struct timerDef_s DEFIO_TIMER_DEF(DEFIO_TIM_ID__ ## tim_); \
    extern struct timerRec_s DEFIO_TIMER_REC(DEFIO_TIM_ID__ ## tim_)    \
    /**/
#endif
