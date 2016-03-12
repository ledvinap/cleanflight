/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SPKY" // SParKY


// MPU 9150 INT connected to PA15, pulled up to VCC by 10K Resistor, contains MPU6050 and AK8975 in single component.
#define GYRO
#define USE_GYRO_MPU6050

#define GYRO_MPU6050_ALIGN CW270_DEG

#define ACC
#define USE_ACC_MPU6050

#define ACC_MPU6050_ALIGN CW270_DEG

#define BARO
#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8975

#define MAG_AK8975_ALIGN CW180_DEG_FLIP

#define LED0
#define LED1
#define LED0_IO          PB4
#define LED1_IO          PB5
#define BEEPER
#define BEEPER_INVERTED
#define BEEPER_IO        PA6

#define USE_EXTI

#define USB_IO

#define USE_VCP
#define USE_UART1 // Conn 1 - TX (PB6) RX PB7 (AF7)
#define USE_UART2 // Input - RX (PA3)
#define USE_UART3 // Servo out - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT 4

#define UART1_TX_IO         PB6
#define UART1_RX_IO         PB7
#define UART1_GPIO_AF       GPIO_AF_7

#define UART2_TX_IO         PA2         // PA2 - Clashes with PWM6 input.
#define UART2_RX_IO         PA3
#define UART2_GPIO_AF       GPIO_AF_7

#define UART3_TX_IO         PB10
#define UART3_RX_IO         PB11
#define UART3_GPIO_AF       GPIO_AF_7

// Note: PA5 and PA0 are N/C on the sparky - potentially use for ADC or LED STRIP?

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA10/AF4), SCL (PA9/AF4)

#define I2C2_SCL_IO          PA9
#define I2C2_SDA_IO          PA10
#define I2C2_AF              GPIO_AF_4

#define USE_ADC

#define ADC_INSTANCE                ADC2
#define ADC_DMA_CHANNEL             DMA2_Channel1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA2

#define VBAT_ADC_IO                 PA4
#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_IO        PA7
#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_7
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_4

#define BLACKBOX
#define GPS
#define GTUNE
#define DISPLAY
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI
#define SONAR

#define LED_STRIP
#if 1
// LED strip configuration using PWM motor output pin 5.
#define LED_STRIP_TIMER TIM16

#define WS2811_IO                       PA6
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_6 // TIM16_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource6
#define WS2811_TIMER                    TIM16
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#endif

#if 0
// Alternate LED strip pin
// FIXME DMA IRQ Transfer Complete is never called because the  TIM17_DMA_RMP needs to be set in SYSCFG_CFGR1
#define LED_STRIP_TIMER TIM17

#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_7 // TIM17_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource7
#define WS2811_TIMER                    TIM17
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM17
#define WS2811_DMA_CHANNEL              DMA1_Channel7
#define WS2811_IRQ                      DMA1_Channel7_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC7
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH7_HANDLER


#endif

//#define USE_SERIAL_1WIRE

#define S1W_TX_GPIO         GPIOB
#define S1W_TX_PIN          GPIO_Pin_6
#define S1W_RX_GPIO         GPIOB
#define S1W_RX_PIN          GPIO_Pin_7

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_IO PA3

// available IO pins
#define TARGET_IO_PORTA (BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTB (BIT(0)|BIT(1)|BIT(10)|BIT(11)|BIT(14)|BIT(15)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9))

// available timers
#define TARGET_TIMER_TIM1     4
#define TARGET_TIMER_TIM8     4
#define TARGET_TIMER_TIM2     4
#define TARGET_TIMER_TIM3     4
#define TARGET_TIMER_TIM4     4
#define TARGET_TIMER_TIM15    2
#define TARGET_TIMER_TIM16    1
#define TARGET_TIMER_TIM17    1
#define TARGET_TIMER_TIM6     0
#define TARGET_TIMER_TIM7     0
