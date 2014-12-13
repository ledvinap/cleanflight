#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn,    0, Mode_IPD},           // S1_IN
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn,    0, Mode_IPD},           // S2_IN - GPIO_PartialRemap_TIM3
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn,    0, Mode_IPD},           // S3_IN
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn,    0, Mode_IPD},           // S4_IN
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn,    0, Mode_IPD},           // S5_IN
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn,    0, Mode_IPD},           // S6_IN

    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn,    0, GPIO_Mode_AF_PP},    // S1_OUT
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn,    0, GPIO_Mode_AF_PP},    // S2_OUT
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn,    0, GPIO_Mode_AF_PP},    // S3_OUT
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, GPIO_Mode_AF_PP},    // S4_OUT
    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn,    0, GPIO_Mode_AF_PP},    // S5_OUT - GPIO_PartialRemap_TIM3
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn,    0, GPIO_Mode_AF_PP},    // S6_OUT

    { TIM1, GPIOA, Pin_9, TIM_Channel_2, TIM1_CC_IRQn, 0, 0},                  // TIMER
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

void timerInitTarget(void)
{
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
//    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); 
}
