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

#define TARGET_BOARD_IDENTIFIER "AFNA" // AFroNAze - NAZE might be considered misleading on Naze clones like the flip32.
#define USE_HARDWARE_REVISION_DETECTION

#define BOARD_HAS_VOLTAGE_DIVIDER

#define LED0
#define LED1
#define LED0_IO        PB3
#define LED1_IO        PB4

#define BEEPER
#define BEEPER_IO      PA12

// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

#define USE_SPI
#define USE_SPI_DEVICE_2

#define NAZE_SPI_INSTANCE     SPI2
#define NAZE_SPI_CS_GPIO      GPIOB
#define NAZE_SPI_CS_PIN       GPIO_Pin_12
#define NAZE_CS_GPIO_CLK_PERIPHERAL RCC_APB2Periph_GPIOB

// We either have this 16mbit flash chip on SPI or the MPU6500 acc/gyro depending on board revision:
#define M25P16_CS_GPIO        NAZE_SPI_CS_GPIO
#define M25P16_CS_PIN         NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE   NAZE_SPI_INSTANCE

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   NAZE_CS_GPIO_CLK_PERIPHERAL
#define MPU6500_CS_GPIO                  NAZE_SPI_CS_GPIO
#define MPU6500_CS_PIN                   NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE             NAZE_SPI_INSTANCE

#define USE_FLASHFS

#define USE_FLASH_M25P16

//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL

//#define DEBUG_MAG_DATA_READY_INTERRUPT
#define USE_MAG_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU3050
#define USE_GYRO_MPU6050
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500

#define GYRO_MPU3050_ALIGN CW0_DEG
#define GYRO_MPU6050_ALIGN CW0_DEG
#define GYRO_MPU6500_ALIGN CW0_DEG

#define ACC
#define USE_ACC_ADXL345
#define USE_ACC_BMA280
#define USE_ACC_MMA8452
#define USE_ACC_MPU6050
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500

#define ACC_ADXL345_ALIGN CW270_DEG
#define ACC_MPU6050_ALIGN CW0_DEG
#define ACC_MMA8452_ALIGN CW90_DEG
#define ACC_BMA280_ALIGN CW0_DEG
#define ACC_MPU6500_ALIGN CW0_DEG

#define BARO
#define BARO_XCLR_IO   PC13
#define BARO_EOC_IO    PC14
#define USE_BARO_MS5611
#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_HMC5883

#define MAG_HMC5883_ALIGN CW180_DEG

#define SONAR
#define USE_EXTI
#define INVERTER
#define INVERTER_IO    PB2           // PB2 (BOOT1) abused as inverter select GPIO
#define INVERTER_USART USART2
#define DISPLAY

#define USE_USART1
#define USART1_RX_IO PA10
#define USART1_TX_IO PA9
#define USART1_RX_IO_REMAP PB7
#define USART1_TX_IO_REMAP PB6
#define USE_USART2
#define USART2_RX_IO PA3
#define USART2_TX_IO PA2

#define USE_USART3
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT 5
#if 0
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4 // PWM 5
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5 // PWM 6
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6 // PWM 7
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7 // PWM 8
#else
// testing configuration
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 0
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 1
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 2
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 2
#endif

// USART3 only on NAZE32_SP - Flex Port
#define USART3_RX_IO PB11
#define USART3_TX_IO PB10

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define USE_ADC

#define CURRENT_METER_ADC_IO      PB1
#define VBAT_ADC_IO               PA4
#define RSSI_ADC_IO               PA1
#define EXTERNAL1_ADC_IO          PA5

#define GPS

#define LED_STRIP
#define LED_STRIP_TIMER TIM3
#define WS2811_DMA_TC_FLAG           DMA1_FLAG_TC6
#define WS2811_DMA_HANDLER_IDENTIFER DMA1_CH6_HANDLER

#define GPS
#define GTUNE
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
//#define AUTOTUNE
#define USE_SERVOS
#define USE_CLI

//#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_IO   PA3

#define TIME_USE_TIMER    // base timming on some timer instead of systick
#define TIME_TIMER 2

//#define USE_SERIAL_1WIRE

// STM32F103CBT6-LQFP48 Pin30 (PA9) TX - PC3 connects to onboard CP2102 RX
#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
// STM32F103CBT6-LQFP48 Pin31 (PA10) RX - PC1 to onboard CP2102 TX
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10

// alternative defaults for AlienWii32 F1 target
#ifdef ALIENWII32
#undef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "AWF1" // AlienWii32 F1.
#undef BOARD_HAS_VOLTAGE_DIVIDER
#define HARDWARE_BIND_PLUG
#define BINDPLUG_IO  PB5               // Hardware bind plug at PB5 (Pin 41)
#endif

#define TARGET_IO_PORTA 0xffff  // TODO!
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff

#define TARGET_TIMER_TIM1     4
#define TARGET_TIMER_TIM2     4
#define TARGET_TIMER_TIM3     4
#define TARGET_TIMER_TIM4     4
