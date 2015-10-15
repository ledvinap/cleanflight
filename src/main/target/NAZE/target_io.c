#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/timer_def.h"
#include "drivers/io_def.h"
#include "drivers/io.h"

const timerChDef_t timerChannelMap[] = {
    DEF_TIMCH(PA0,  2, 1), // IO1   PPM    PWM1  PPM   PWM1    ;3 ADC12_IN0              ; 1
    DEF_TIMCH(PA1,  2, 2), // IO2          PWM2        PWM2    ;3 ADC12_IN1
    DEF_TIMCH(PA2,  2, 3), // IO3          PWM3        PWM3    ;3 ADC12_IN2 USART2_TX
    DEF_TIMCH(PA3,  2, 4), // IO4          PWM4        PWM4    ;3 ADC12_IN3 USART2_RX
    DEF_TIMCH(PA6,  3, 1), // IO5   OUT7   PWM5  OUT5  PWM5    ;3 ADC12_IN6
    DEF_TIMCH(PA7,  3, 2), // IO6   OUT8   PWM6  OUT6  PWM6    ;3 ADC12_IN7
    DEF_TIMCH(PB0,  3, 3), // IO7   OUT9   PWM7  OUT7  PWM7    ;3 ADC12_IN8
    DEF_TIMCH(PB1,  3, 4), // IO8   OUT10  PWM8  OUT8  PWM8    ;3 ADC12_IN9
    DEF_TIMCH(PA8,  1, 1), // IO9   OUT1   OUT1  MOT1  MOT1    ;5                        ; 2
    DEF_TIMCH(PA11, 1, 4), // IO10  OUT2   OUT2  MOT2  MOT2    ;5                        ; (M2 pad)
    DEF_TIMCH(PB6,  4, 1), // IO11  OUT3   OUT3  OUT1  OUT1    ;5 I2C1_SCL USART1_TX     ; 3
    DEF_TIMCH(PB7,  4, 2), // IO12  OUT4   OUT4  OUT2  OUT2    ;5 I2C1_SDA USART1_RX     ; 4
    DEF_TIMCH(PB8,  4, 3), // IO13  OUT5   OUT5  OUT3  OUT3    ;5 I2C1_SCL CANRX        ; 5
    DEF_TIMCH(PB9,  4, 4), // IO14  OUT6   OUT6  OUT4  OUT4    ;5 I2C1_SDA CANTX        ; 6
    DEF_TIMCH(PA9,  1, 2), // IO15                             ;5 USART1_TX
    DEF_TIMCH(PA10, 1, 3), // IO16                             ;5 USART1_TX
};

int timerChannelMap_Count(void)
{
    return ARRAYLEN(timerChannelMap);
}
