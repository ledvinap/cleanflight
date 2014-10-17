#ifndef INCLUDE_LAYOUT_BODY
// this part is included in header
#define USABLE_TIMER_CHANNEL_COUNT 14

#else
// this will be included in timer.c
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

#define USED_TIMERS  (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(15)|TIM_N(16)|TIM_N(17))


#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)
