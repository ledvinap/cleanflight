#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,    0, Mode_IPD}, // PWM1   PPM    PWM1  PPM   PWM1
    { TIM2, GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,    0, Mode_IPD}, // PWM2          PWM2        PWM2
    { TIM2, GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,    0, Mode_IPD}, // PWM3          PWM3        PWM3
    { TIM2, GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,    0, Mode_IPD}, // PWM4          PWM4        PWM4
    { TIM3, GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,    0, Mode_IPD}, // PWM5   OUT7   PWM5  OUT5  PWM5
    { TIM3, GPIOA, Pin_7,  TIM_Channel_2, TIM3_IRQn,    0, Mode_IPD}, // PWM6   OUT8   PWM6  OUT6  PWM6
    { TIM3, GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,    0, Mode_IPD}, // PWM7   OUT9   PWM7  OUT7  PWM7
    { TIM3, GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,    0, Mode_IPD}, // PWM8   OUT10  PWM8  OUT8  PWM8
    { TIM1, GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD}, // PWM9   OUT1   OUT1  MOT1  MOT1
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD}, // PWM10  OUT2   OUT2  MOT2  MOT2
    { TIM4, GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,    0, Mode_IPD}, // PWM11  OUT3   OUT3  OUT1  OUT1
    { TIM4, GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,    0, Mode_IPD}, // PWM12  OUT4   OUT4  OUT2  OUT2
    { TIM4, GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,    0, Mode_IPD}, // PWM13  OUT5   OUT5  OUT3  OUT3
    { TIM4, GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,    0, Mode_IPD}, // PWM14  OUT6   OUT6  OUT4  OUT4

//    { TIM1, GPIOA, Pin_10,  TIM_Channel_3, TIM1_CC_IRQn, 0, 0},        // TIMER
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

void timerInitTarget(void)
{
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
//    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
}
