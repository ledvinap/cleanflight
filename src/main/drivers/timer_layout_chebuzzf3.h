#ifndef INCLUDE_LAYOUT_BODY
// this part is included in header
#define USABLE_TIMER_CHANNEL_COUNT 14

#else
// this will be included in timer.c

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

#define USED_TIMERS  (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(15)|TIM_N(16)|TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF)
