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

#define TARGET_BOARD_IDENTIFIER "CFLY"

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_4 // PB4 (RED LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOB
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_5 // PB5 (GREEN LED)
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOB


// SPI2
// PB15 28 SPI2_MOSI
// PB14 27 SPI2_MISO
// PB13 26 SPI2_SCK
// PB12 25 SPI2_NSS

//#define USE_SPI
//#define USE_SPI_DEVICE_2

//#define NAZE_SPI_INSTANCE     SPI2
//#define NAZE_SPI_CS_GPIO      GPIOB
//#define NAZE_SPI_CS_PIN       GPIO_Pin_12

//#define MPU6500_CS_GPIO       NAZE_SPI_CS_GPIO
//#define MPU6500_CS_PIN        NAZE_SPI_CS_PIN
//#define MPU6500_SPI_INSTANCE  NAZE_SPI_INSTANCE

#define GYRO
#define USE_GYRO_MPU6050

#define ACC
#define USE_ACC_MPU6050

#define BARO
#define USE_BARO_MS5611

#define MAG
#define LED0
#define LED1

#define USE_VCP
#define USE_USART3
#define USE_SOFTSERIAL
#define SERIAL_PORT_COUNT 3

#define USART3_TX_PIN Pin_10
#define USART3_RX_PIN Pin_11
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

#define TELEMETRY
#define SERIAL_RX
#define AUTOTUNE

#define USABLE_TIMER_CHANNEL_COUNT (9 + 1)
