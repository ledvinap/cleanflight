#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/io_def.h"
#include "drivers/io.h"


const timerChDef_t timerChDefs[] = {       //                                afromini
    DEF_TIMCH(GPIOA, PIN0,  TIM2, TIMCH1), // IO1   PPM    PWM1  PPM   PWM1    ;3 ADC12_IN0              ; 1
    DEF_TIMCH(GPIOA, PIN1,  TIM2, TIMCH2), // IO2          PWM2        PWM2    ;3 ADC12_IN1
    DEF_TIMCH(GPIOA, PIN2,  TIM2, TIMCH3), // IO3          PWM3        PWM3    ;3 ADC12_IN2 USART2_TX
    DEF_TIMCH(GPIOA, PIN3,  TIM2, TIMCH4), // IO4          PWM4        PWM4    ;3 ADC12_IN3 USART2_RX
    DEF_TIMCH(GPIOA, PIN6,  TIM3, TIMCH1), // IO5   OUT7   PWM5  OUT5  PWM5    ;3 ADC12_IN6
    DEF_TIMCH(GPIOA, PIN7,  TIM3, TIMCH2), // IO6   OUT8   PWM6  OUT6  PWM6    ;3 ADC12_IN7
    DEF_TIMCH(GPIOB, PIN0,  TIM3, TIMCH3), // IO7   OUT9   PWM7  OUT7  PWM7    ;3 ADC12_IN8
    DEF_TIMCH(GPIOB, PIN1,  TIM3, TIMCH4), // IO8   OUT10  PWM8  OUT8  PWM8    ;3 ADC12_IN9
    DEF_TIMCH(GPIOA, PIN8,  TIM1, TIMCH1), // IO9   OUT1   OUT1  MOT1  MOT1    ;5                        ; 2
    DEF_TIMCH(GPIOA, PIN11, TIM1, TIMCH4), // IO10  OUT2   OUT2  MOT2  MOT2    ;5                        ; (M2 pad)
    DEF_TIMCH(GPIOB, PIN6,  TIM4, TIMCH1), // IO11  OUT3   OUT3  OUT1  OUT1    ;5 I2C1_SCL USART1_TX     ; 3
    DEF_TIMCH(GPIOB, PIN7,  TIM4, TIMCH2), // IO12  OUT4   OUT4  OUT2  OUT2    ;5 I2C1_SDA USART1_RX     ; 4
    DEF_TIMCH(GPIOB, PIN8,  TIM4, TIMCH3), // IO13  OUT5   OUT5  OUT3  OUT3    ;5 I2C1_SCL CANRX        ; 5
    DEF_TIMCH(GPIOB, PIN9,  TIM4, TIMCH4), // IO14  OUT6   OUT6  OUT4  OUT4    ;5 I2C1_SDA CANTX        ; 6
    DEF_TIMCH(GPIOA, PIN9,  TIM1, TIMCH2), // IO15                             ;5 USART1_TX
    DEF_TIMCH(GPIOA, PIN10, TIM1, TIMCH3), // IO16                             ;5 USART1_TX
};

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
    timerChDefs + 11,
    timerChDefs + 12,
    timerChDefs + 13,
    timerChDefs + 14,
    timerChDefs + 15,
};


void timerInitTarget(void)
{

}
