#pragma once

// define macros for easy specification of IO pins.

// Structure defining available IO pins is defined using DEF_IO macro. Parameters to this macro are intended to be easily readable.
// The paramaters are concatenated before expansion, so no namespace collision should occut (we can use TIMx, etc. as parameter)
// this also allows using macro parameters in differenct context. This mechanism may be improved if boost preprocessor (BOOST PP) is 
// used in future.

// PORTx to GPIOx mapping
#define DEFIO_GPIO__PORTA GPIOA
#define DEFIO_GPIO__PORTB GPIOB
#define DEFIO_GPIO__PORTC GPIOC

// PINx to Pin_x
#define DEFIO_Pin__PIN0 Pin_0
#define DEFIO_Pin__PIN1 Pin_1
#define DEFIO_Pin__PIN2 Pin_2
#define DEFIO_Pin__PIN3 Pin_3
#define DEFIO_Pin__PIN4 Pin_4
#define DEFIO_Pin__PIN5 Pin_5
#define DEFIO_Pin__PIN6 Pin_6
#define DEFIO_Pin__PIN7 Pin_7
#define DEFIO_Pin__PIN8 Pin_8
#define DEFIO_Pin__PIN9 Pin_9
#define DEFIO_Pin__PIN10 Pin_10
#define DEFIO_Pin__PIN11 Pin_11
#define DEFIO_Pin__PIN12 Pin_12
#define DEFIO_Pin__PIN13 Pin_13
#define DEFIO_Pin__PIN14 Pin_14
#define DEFIO_Pin__PIN15 Pin_15

// TIMx to TIMx mapping (identity now)
#define DEFIO_TIM__NA NULL
#define DEFIO_TIM__TIM1 TIM1
#define DEFIO_TIM__TIM2 TIM2
#define DEFIO_TIM__TIM3 TIM3
#define DEFIO_TIM__TIM4 TIM4

// TIMx to TIMER_INDEX(x)
#define DEFIO_TIMER_INDEX__NA 0xff
#define DEFIO_TIMER_INDEX__TIM1 TIMER_INDEX(1)
#define DEFIO_TIMER_INDEX__TIM2 TIMER_INDEX(2)
#define DEFIO_TIMER_INDEX__TIM3 TIMER_INDEX(3)
#define DEFIO_TIMER_INDEX__TIM4 TIMER_INDEX(4)

// TIMCH to TIM_Channel_x mapping
#define DEFIO_TIM_Channel__NA 0
#define DEFIO_TIM_Channel__TIMCH1 TIM_Channel_1
#define DEFIO_TIM_Channel__TIMCH2 TIM_Channel_2
#define DEFIO_TIM_Channel__TIMCH3 TIM_Channel_3
#define DEFIO_TIM_Channel__TIMCH4 TIM_Channel_4

// the ## operator must be used directly here - the parameter should not be expanded first
// some magic may be used if expansion is desirable (#define DEFIO_GPIO__EXPAND(x) CONCAT(GPIO, x) )
#define DEF_IO(port, pin, tim, tim_ch) { DEFIO_TIM__ ## tim, DEFIO_GPIO__ ## port, DEFIO_Pin__ ## pin, DEFIO_TIM_Channel__ ## tim_ch, DEFIO_TIMER_INDEX__ ## tim }

// macros neccessary to define IO structure. Maybe use #include in target_io.c instead

#include "drivers/timer_impl.h"
