#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

const timerHardware_t timerHardware[USABLE_IO_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,    0, Mode_IPD}, // PIN1   PPM    PWM1  PPM   PWM1    ; ADC12_IN0
    { TIM2, GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,    0, Mode_IPD}, // PIN2          PWM2        PWM2    ; ADC12_IN1
    { TIM2, GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,    0, Mode_IPD}, // PIN3          PWM3        PWM3    ; ADC12_IN2 USART2_TX
    { TIM2, GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,    0, Mode_IPD}, // PIN4          PWM4        PWM4    ; ADC12_IN3 USART2_RX
    { TIM3, GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,    0, Mode_IPD}, // PIN5   OUT7   PWM5  OUT5  PWM5    ; ADC12_IN6
    { TIM3, GPIOA, Pin_7,  TIM_Channel_2, TIM3_IRQn,    0, Mode_IPD}, // PIN6   OUT8   PWM6  OUT6  PWM6    ; ADC12_IN7
    { TIM3, GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,    0, Mode_IPD}, // PIN7   OUT9   PWM7  OUT7  PWM7    ; ADC12_IN8
    { TIM3, GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,    0, Mode_IPD}, // PIN8   OUT10  PWM8  OUT8  PWM8    ; ADC12_IN9
    { TIM1, GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD}, // PIN9   OUT1   OUT1  MOT1  MOT1    ;
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD}, // PIN10  OUT2   OUT2  MOT2  MOT2    ;
    { TIM4, GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,    0, Mode_IPD}, // PIN11  OUT3   OUT3  OUT1  OUT1    ; I2C1_SCL USART1_TX
    { TIM4, GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,    0, Mode_IPD}, // PIN12  OUT4   OUT4  OUT2  OUT2    ; I2C1_SDA USART1_RX
    { TIM4, GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,    0, Mode_IPD}, // PIN13  OUT5   OUT5  OUT3  OUT3    ; I2C1_SCL CANRX
    { TIM4, GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,    0, Mode_IPD}, // PIN14  OUT6   OUT6  OUT4  OUT4    ; I2C1_SDA CANTX
    { TIM1, GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn, 1, Mode_IPD}, // PIN15  ; USART1_TX
    { TIM1, GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 1, Mode_IPD}, // PIN16  ; USART1_TX
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

void timerInitTarget(void)
{
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
//    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
}
