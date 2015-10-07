#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/timer_def.h"
#include "drivers/io_def.h"
#include "drivers/io.h"

// exposed channel map, used to init mixer
// TODO - change to list of available pins, let user decide how to use them
const timerChDef_t timerChannelMap[] = {
    // 6x3 pin header, PWM 1-6
    DEF_TIMCH(PB15, 15,  2, GPIO_AF_1), // PWM1  - PB15 - TIM1_CH3N, TIM15_CH1N, *TIM15_CH2
    DEF_TIMCH(PB14, 15,  1, GPIO_AF_1), // PWM2  - PB14 - TIM1_CH2N, *TIM15_CH1
    DEF_TIMCH(PA8,   1,  1, GPIO_AF_6), // PWM3  - PA8  - *TIM1_CH1, TIM4_ETR
    DEF_TIMCH(PB0,   3,  3, GPIO_AF_2), // PWM4  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIMCH(PA6,   3,  1, GPIO_AF_2), // PWM5  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    DEF_TIMCH(PA2,   2,  3, GPIO_AF_1), // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1

    // 6 pin header, PWM7-10
    DEF_TIMCH(PB1,   3,  4, GPIO_AF_2), // PWM7  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    DEF_TIMCH(PA7,  17,  1, GPIO_AF_1), // PWM8  - PA7  - !TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    DEF_TIMCH(PA4,   3,  2, GPIO_AF_2), // PWM9  - PA4  - *TIM3_CH2
    DEF_TIMCH(PA1,   2,  2, GPIO_AF_1), // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N

    //
    // PPM PORT - Also USART2 RX (AF5)
    //

    DEF_TIMCH(PA3,  2,  4, GPIO_AF_1), // PPM   - PA3  - TIM2_CH4, TIM15_CH2 - PWM13

    // USART3 RX/TX
    // RX conflicts with PPM port
    //{ 2,  GPIOB, Pin_11, TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource11,  GPIO_AF_1} // RX    - PB11 - *TIM2_CH4, USART3_RX (AF7) - PWM11
    //{ 2,  GPIOB, Pin_10, TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource10,  GPIO_AF_1} // TX    - PB10 - *TIM2_CH3, USART3_TX (AF7) - PWM12

};

// channel user for timerQueue - must not be skipped and must count to 0xffff at 1MHz
const timerChDef_t timerQueueChDef =
    DEF_TIMCH(NONE,  2,  2, 0); // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N

