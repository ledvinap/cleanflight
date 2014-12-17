#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3, GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,    0, Mode_IPD}, // MOTOR1
    { TIM3, GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,    0, Mode_IPD}, // MOTOR2
    { TIM4, GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,    0, Mode_IPD}, // MOTOR3
    { TIM4, GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,    0, Mode_IPD}, // MOTOR4

    { TIM3, GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,    0, Mode_IPD}, // EXT_SPI_MISO
    { TIM3, GPIOA, Pin_7,  TIM_Channel_2, TIM3_IRQn,    0, Mode_IPD}, // EXT_SPI_MOSI

    { TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn,    0, Mode_IPD}, // EXT_I2C_SCL/TX (timer after remap)
    { TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,    0, Mode_IPD}, // EXT_I2C_SDA/RX (timer after remap)

    { TIM2, GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn,    0, Mode_IPD}, // Jtag TDO ; TIM2CH2 after remap

    { TIM1, GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 0, 0},        // TIMER
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

void timerInitTarget(void)
{
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
//    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
}
