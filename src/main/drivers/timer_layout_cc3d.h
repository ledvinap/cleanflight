#ifndef INCLUDE_LAYOUT_BODY
// this part is included in header
#define USABLE_TIMER_CHANNEL_COUNT 14

#else
// this will be included in timer.c
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

#define USED_TIMERS  (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
