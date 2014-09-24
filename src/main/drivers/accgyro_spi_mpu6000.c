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


/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "bus_spi.h"

#include "accgyro.h"
#include "accgyro_spi_mpu6000.h"

// Registers
#define MPU6000_PRODUCT_ID      	0x0C
#define MPU6000_SMPLRT_DIV	    	0x19
#define MPU6000_GYRO_CONFIG	    	0x1B
#define MPU6000_ACCEL_CONFIG  		0x1C
#define MPU6000_FIFO_EN		    	0x23
#define MPU6000_INT_PIN_CFG	    	0x37
#define MPU6000_INT_ENABLE	    	0x38
#define MPU6000_INT_STATUS	    	0x3A
#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48
#define MPU6000_USER_CTRL	    	0x6A
#define MPU6000_SIGNAL_PATH_RESET   0x68
#define MPU6000_PWR_MGMT_1	    	0x6B
#define MPU6000_PWR_MGMT_2	    	0x6C
#define MPU6000_FIFO_COUNTH	    	0x72
#define MPU6000_FIFO_COUNTL	    	0x73
#define MPU6000_FIFO_R_W		   	0x74
#define MPU6000_WHOAMI		    	0x75

// Bits
#define BIT_SLEEP				    0x40
#define BIT_H_RESET				    0x80
#define BITS_CLKSEL				    0x07
#define MPU_CLK_SEL_PLLGYROX	    0x01
#define MPU_CLK_SEL_PLLGYROZ	    0x03
#define MPU_EXT_SYNC_GYROX		    0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN			    0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA		    0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

#ifdef CC3D

#define MPU6000_CS_GPIO       GPIOA
#define MPU6000_CS_PIN        GPIO_Pin_4

#define MPU6000_SPI_INSTANCE SPI1
#endif

#define DISABLE_MPU6000       GPIO_SetBits(MPU6000_CS_GPIO,   MPU6000_CS_PIN)
#define ENABLE_MPU6000        GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN)

void mpu6000SpiGyroRead(int16_t *gyroData);
void mpu6000SpiAccRead(int16_t *gyroData);

void mpu6000SpiGyroInit(void)
{
}

void mpu6000SpiAccInit(void)
{
    acc_1G = 512 * 8;
}

bool mpu6000SpiDetect(void)
{
    // FIXME this isn't working, not debugged yet.
    return true; // just assume it's there for now
#if 0
    uint8_t product;

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_PRODUCT_ID);
    spiTransfer(MPU6000_SPI_INSTANCE, &product, NULL, 1);
    DISABLE_MPU6000;




    /* look for a product ID we recognise */

    // verify product revision
    switch (product) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return false;
#endif
}

static bool initDone = false;

void mpu6000AccAndGyroInit() {

    if (initDone) {
        return;
    }

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_PWR_MGMT_1);          // Device Reset
    spiTransferByte(MPU6000_SPI_INSTANCE, BIT_H_RESET);
    DISABLE_MPU6000;

    delay(150);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_SIGNAL_PATH_RESET);          // Device Reset
    spiTransferByte(MPU6000_SPI_INSTANCE, BIT_GYRO | BIT_ACC | BIT_TEMP);
    DISABLE_MPU6000;

    delay(150);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_PWR_MGMT_1);          // Clock Source PPL with Z axis gyro reference
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU_CLK_SEL_PLLGYROZ);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_USER_CTRL);           // Disable Primary I2C Interface
    spiTransferByte(MPU6000_SPI_INSTANCE, BIT_I2C_IF_DIS);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_PWR_MGMT_2);
    spiTransferByte(MPU6000_SPI_INSTANCE, 0x00);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_SMPLRT_DIV);          // Accel Sample Rate 1kHz
    spiTransferByte(MPU6000_SPI_INSTANCE, 0x00);                        // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_ACCEL_CONFIG);        // Accel +/- 8 G Full Scale
    spiTransferByte(MPU6000_SPI_INSTANCE, BITS_FS_8G);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_GYRO_CONFIG);         // Gyro +/- 1000 DPS Full Scale
    spiTransferByte(MPU6000_SPI_INSTANCE, BITS_FS_2000DPS);
    DISABLE_MPU6000;

    initDone = true;
}

bool mpu6000SpiAccDetect(acc_t *acc)
{
    if (!mpu6000SpiDetect()) {
        return false;
    }

    spiResetErrorCounter(MPU6000_SPI_INSTANCE);

    mpu6000AccAndGyroInit();

    acc->init = mpu6000SpiAccInit;
    acc->read = mpu6000SpiAccRead;

    delay(100);
    return true;
}

bool mpu6000SpiGyroDetect(gyro_t *gyro, uint16_t lpf)
{
    if (!mpu6000SpiDetect()) {
        return false;
    }

    spiResetErrorCounter(MPU6000_SPI_INSTANCE);

    mpu6000AccAndGyroInit();

    uint8_t mpuLowPassFilter = BITS_DLPF_CFG_42HZ;
    int16_t data[3];

    // default lpf is 42Hz
    switch (lpf) {
        case 256:
            mpuLowPassFilter = BITS_DLPF_CFG_256HZ;
            break;
        case 188:
            mpuLowPassFilter = BITS_DLPF_CFG_188HZ;
            break;
        case 98:
            mpuLowPassFilter = BITS_DLPF_CFG_98HZ;
            break;
        default:
        case 42:
            mpuLowPassFilter = BITS_DLPF_CFG_42HZ;
            break;
        case 20:
            mpuLowPassFilter = BITS_DLPF_CFG_20HZ;
            break;
        case 10:
            mpuLowPassFilter = BITS_DLPF_CFG_10HZ;
            break;
        case 5:
            mpuLowPassFilter = BITS_DLPF_CFG_5HZ;
            break;
        case 0:
            mpuLowPassFilter = BITS_DLPF_CFG_2100HZ_NOLPF;
            break;
    }

    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_0_5625MHZ_CLOCK_DIVIDER);

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_CONFIG);              // Accel and Gyro DLPF Setting
    spiTransferByte(MPU6000_SPI_INSTANCE, mpuLowPassFilter);
    DISABLE_MPU6000;

    delayMicroseconds(1);

    mpu6000SpiGyroRead(data);

    if ((((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) || spiGetErrorCounter(MPU6000_SPI_INSTANCE) != 0) {
        spiResetErrorCounter(MPU6000_SPI_INSTANCE);
        return false;
    }
    gyro->init = mpu6000SpiGyroInit;
    gyro->read = mpu6000SpiGyroRead;
    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;
    //gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;
    delay(100);
    return true;
}

void mpu6000SpiGyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_GYRO_XOUT_H | 0x80);
    spiTransfer(MPU6000_SPI_INSTANCE, buf, NULL, 6);
    DISABLE_MPU6000;

    gyroData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[Z] = (int16_t)((buf[4] << 8) | buf[5]);
}

void mpu6000SpiAccRead(int16_t *gyroData)
{
    uint8_t buf[6];
    spiSetDivisor(MPU6000_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);  // 18 MHz SPI clock

    ENABLE_MPU6000;
    spiTransferByte(MPU6000_SPI_INSTANCE, MPU6000_ACCEL_XOUT_H | 0x80);
    spiTransfer(MPU6000_SPI_INSTANCE, buf, NULL, 6);
    DISABLE_MPU6000;

    gyroData[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[Z] = (int16_t)((buf[4] << 8) | buf[5]);
}
