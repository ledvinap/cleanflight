/* FreeFlight/Naze32 timer layout
    TIM2_CH1    RC1             PWM1
    TIM2_CH2    RC2             PWM2
    TIM2_CH3    RC3/UA2_TX      PWM3
    TIM2_CH4    RC4/UA2_RX      PWM4
    TIM3_CH1    RC5             PWM5
    TIM3_CH2    RC6             PWM6
    TIM3_CH3    RC7             PWM7
    TIM3_CH4    RC8             PWM8
    TIM1_CH1    PWM1            PWM9
    TIM1_CH4    PWM2            PWM10
    TIM4_CH1    PWM3            PWM11
    TIM4_CH2    PWM4            PWM12
    TIM4_CH3    PWM5            PWM13
    TIM4_CH4    PWM6            PWM14

    RX1  TIM2_CH1 PA0 [also PPM] [also used for throttle calibration]
    RX2  TIM2_CH2 PA1
    RX3  TIM2_CH3 PA2 [also UART2_TX]
    RX4  TIM2_CH4 PA3 [also UART2_RX]
    RX5  TIM3_CH1 PA6 [also ADC_IN6]
    RX6  TIM3_CH2 PA7 [also ADC_IN7]
    RX7  TIM3_CH3 PB0 [also ADC_IN8]
    RX8  TIM3_CH4 PB1 [also ADC_IN9]

    Outputs
    PWM1 TIM1_CH1 PA8
    PWM2 TIM1_CH4 PA11
    PWM3 TIM4_CH1 PB6 [also I2C1_SCL]
    PWM4 TIM4_CH2 PB7 [also I2C1_SDA]
    PWM5 TIM4_CH3 PB8
    PWM6 TIM4_CH4 PB9

    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM2 4 channels
    TIM3 4 channels
    TIM1 2 channels
    TIM4 4 channels
*/

#ifndef INCLUDE_LAYOUT_BODY
// this part is included in header
#define USABLE_TIMER_CHANNEL_COUNT (14+1)
#define TIMER_QUEUE_CHANNEL (USABLE_TIMER_CHANNEL_COUNT-1)

#else
// this will be included in timer.c

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD},          // PWM1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD},          // PWM2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_IPD},          // PWM3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_IPD},          // PWM4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, Mode_IPD},          // PWM5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD},          // PWM6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD},          // PWM7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD},          // PWM8
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD},       // PWM9
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD},      // PWM10
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD},          // PWM11
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, Mode_IPD},          // PWM12
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, Mode_IPD},          // PWM13
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, Mode_IPD},          // PWM14
    { TIM1, GPIOA, Pin_9, TIM_Channel_2, TIM1_CC_IRQn, 0, 0},              // TIMER
};


#define USED_TIMERS             (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#endif
