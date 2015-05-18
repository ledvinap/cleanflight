#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/io_def.h"
#include "drivers/io.h"

const timerChDef_t timerChDefs[] = { 
    DEF_TIMCH(GPIOB, PIN15, TIM15, TIMCH2), // PWM1  - PB15 - TIM1_CH3N, TIM15_CH1N, *TIM15_CH2
    DEF_TIMCH(GPIOB, PIN14, TIM15, TIMCH1), // PWM2  - PB14 - TIM1_CH2N, *TIM15_CH1
    DEF_TIMCH(GPIOA, PIN8,  TIM1,  TIMCH1), // PWM3  - PA8  - *TIM1_CH1, TIM4_ETR
    DEF_TIMCH(GPIOB, PIN0,  TIM3,  TIMCH3), // PWM4  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    DEF_TIMCH(GPIOA, PIN6,  TIM3,  TIMCH1), // PWM5  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    DEF_TIMCH(GPIOA, PIN2,  TIM2,  TIMCH3), // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1

    //
    // 6 pin header
    //

    // PWM7-10
    DEF_TIMCH(GPIOB, PIN1,  TIM3,  TIMCH4), // PWM7  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    DEF_TIMCH(GPIOA, PIN7,  TIM17, TIMCH1), // PWM8  - PA7  - !TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    DEF_TIMCH(GPIOA, PIN4,  TIM3,  TIMCH2), // PWM9  - PA4  - *TIM3_CH2
    DEF_TIMCH(GPIOA, PIN1,  TIM2,  TIMCH2), // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N

    //
    // PPM PORT - Also USART2 RX (AF5)
    //

    DEF_TIMCH(GPIOA, PIN3,  TIM2,  TIMCH4), // PPM   - PA3  - TIM2_CH4, TIM15_CH2 - PWM13

    // USART3 RX/TX
    // RX conflicts with PPM port
    //{ TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource11,  GPIO_AF_1} // RX    - PB11 - *TIM2_CH4, USART3_RX (AF7) - PWM11
    //{ TIM2,  GPIOB, Pin_10, TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource10,  GPIO_AF_1} // TX    - PB10 - *TIM2_CH3, USART3_TX (AF7) - PWM12

};

const timerChDef_t timerQueueChannelDef = 
    DEF_TIMCH(GPIOA, PIN1,  TIM2,  TIMCH2); // PWM10 - PA1  - *TIM2_CH2, TIM15_CH1N

// todo!
const timerChDef_t* const timerChannelMap[USABLE_IO_CHANNEL_COUNT] = {
    timerChDefs + 0,
    timerChDefs + 1,
    timerChDefs + 2,
    timerChDefs + 3,
    timerChDefs + 4,
    timerChDefs + 5,
    timerChDefs + 6,
    timerChDefs + 7,
    timerChDefs + 8,
    timerChDefs + 9,
    timerChDefs + 10,
};



void timerInitTarget(void)
{
}
