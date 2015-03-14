#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/io_def.h"


const timerHardware_t timerHardware[USABLE_IO_CHANNEL_COUNT] = {      //                                afromini
    DEF_IO(PORTA, PIN0,  TIM2, TIMCH1), // IO1   PPM    PWM1  PPM   PWM1    ;3 ADC12_IN0              ; 1
    DEF_IO(PORTA, PIN1,  TIM2, TIMCH2), // IO2          PWM2        PWM2    ;3 ADC12_IN1
    DEF_IO(PORTA, PIN2,  TIM2, TIMCH3), // IO3          PWM3        PWM3    ;3 ADC12_IN2 USART2_TX
    DEF_IO(PORTA, PIN3,  TIM2, TIMCH4), // IO4          PWM4        PWM4    ;3 ADC12_IN3 USART2_RX
    DEF_IO(PORTA, PIN6,  TIM3, TIMCH1), // IO5   OUT7   PWM5  OUT5  PWM5    ;3 ADC12_IN6
    DEF_IO(PORTA, PIN7,  TIM3, TIMCH2), // IO6   OUT8   PWM6  OUT6  PWM6    ;3 ADC12_IN7
    DEF_IO(PORTB, PIN0,  TIM3, TIMCH3), // IO7   OUT9   PWM7  OUT7  PWM7    ;3 ADC12_IN8
    DEF_IO(PORTB, PIN1,  TIM3, TIMCH4), // IO8   OUT10  PWM8  OUT8  PWM8    ;3 ADC12_IN9
    DEF_IO(PORTA, PIN8,  TIM1, TIMCH1), // IO9   OUT1   OUT1  MOT1  MOT1    ;5                        ; 2
    DEF_IO(PORTA, PIN11, TIM1, TIMCH4), // IO10  OUT2   OUT2  MOT2  MOT2    ;5                        ; (M2 pad)
    DEF_IO(PORTB, PIN6,  TIM4, TIMCH1), // IO11  OUT3   OUT3  OUT1  OUT1    ;5 I2C1_SCL USART1_TX     ; 3
    DEF_IO(PORTB, PIN7,  TIM4, TIMCH2), // IO12  OUT4   OUT4  OUT2  OUT2    ;5 I2C1_SDA USART1_RX     ; 4
    DEF_IO(PORTB, PIN8,  TIM4, TIMCH3), // IO13  OUT5   OUT5  OUT3  OUT3    ;5 I2C1_SCL CANRX        ; 5
    DEF_IO(PORTB, PIN9,  TIM4, TIMCH4), // IO14  OUT6   OUT6  OUT4  OUT4    ;5 I2C1_SDA CANTX        ; 6
    DEF_IO(PORTA, PIN9,  TIM1, TIMCH2), // IO15                             ;5 USART1_TX
    DEF_IO(PORTA, PIN10, TIM1, TIMCH3), // IO16                             ;5 USART1_TX

    DEF_IO(PORTC, PIN13, NA,   NA),     // IO17  BARO XCLR
    DEF_IO(PORTC, PIN14, NA,   NA),     // IO18  BARO EOC
};

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC)

void timerInitTarget(void)
{
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
//    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
}
