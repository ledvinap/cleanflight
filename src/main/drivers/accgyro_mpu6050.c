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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/pin_debug.h"
#include "drivers/exti.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6050.h"

extern uint8_t mpuLowPassFilter;

//#define DEBUG_MPU_DATA_READY_INTERRUPT

// MPU6050, Standard address 0x68
// MPU_INT on PB13 on rev4 Naze32 hardware
#define MPU6050_ADDRESS         0x68

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F

// RA = Register Address

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

// RF = Register Flag
// MPU_RA_FIFO_EN
#define MPU_RF_TEMP_FIFO_EN     0x80
#define MPU_RF_XG_FIFO_EN       0x40
#define MPU_RF_YG_FIFO_EN       0x20
#define MPU_RF_ZG_FIFO_EN       0x10
#define MPU_RF_ACCEL_FIFO_EN    0x08
#define MPU_RF_SLV2_FIFO_EN     0x04
#define MPU_RF_SLV1_FIFO_EN     0x02
#define MPU_RF_SLV0_FIFO_EN     0x01
// MPU_RA_USER_CTRL
#define MPU_RF_FIFO_EN          0x40
#define MPU_RF_I2C_MST_EN       0x20
#define MPU_RF_I2C_IF_DIS       0x10
#define MPU_RF_FIFO_RESET       0x04
#define MPU_RF_I2C_MST_RESET    0x02
#define MPU_RF_SIG_COND_RESET   0x01
// MPU_RA_INT_ENABLE
#define MPU_RF_DATA_RDY_EN (1 << 0)

#define MPU6050_SMPLRT_DIV      0       // 8000Hz

static void mpu6050AccInit(void);
static void mpu6050GyroInit(uint16_t lpf);

void mpu6050FifoEnable(void);

static const mpu6050Config_t *mpu6050Config = NULL;

#ifdef USE_MPU_DATA_READY_SIGNAL

static extiCallbackRec_t mpu6050_extiCallbackRec;

void mpu6050_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    // Measure the delta in micro seconds between calls to the interrupt handler
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = micros();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}

#endif

void mpu6050GpioInit(void) {
    static bool mpu6050GpioInitDone = false;

    if (mpu6050GpioInitDone || !mpu6050Config) {
        return;
    }


#ifdef USE_MPU_DATA_READY_SIGNAL
    IO_t intIO = IOGetByTag(mpu6050Config->intIO);
    IOConfigGPIO(intIO, IOCFG_IN_FLOATING);  // TODO - EXTIConfigGPIO() ?
    EXTIHandlerInit(&mpu6050_extiCallbackRec, mpu6050_extiHandler);
    EXTIConfig(intIO, &mpu6050_extiCallbackRec, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(intIO, true);
#endif

    mpu6050GpioInitDone = true;
}

bool mpu6050AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }

    acc->init = mpu6050AccInit;
    acc->read = mpuAccRead;
    acc->revisionCode = (mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n'); // es/non-es variance between MPU6050 sensors, half of the naze boards are mpu6000ES.

    return true;
}

bool mpu6050GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }
    gyro->init = mpu6050GyroInit;
    gyro->read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

static void mpu6050AccInit(void)
{
    mpuIntExtiInit();

    switch (mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            acc_1G = 256 * 8;
            break;
        case MPU_FULL_RESOLUTION:
            acc_1G = 512 * 8;
            break;
    }
}

static void mpu6050GyroInit(uint16_t lpf)
{
    bool ack;

    mpuIntExtiInit();

    uint8_t mpuLowPassFilter = determineMPULPF(lpf);

    ack = mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(100);
    ack = mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    delay(15); //PLL Settling time when changing CLKSEL is max 10ms.  Use 15ms to be sure 
    if(mpuLowPassFilter == INV_FILTER_256HZ_NOLPF2
       || mpuLowPassFilter == INV_FILTER_2100HZ_NOLPF)          // keep 1khz sampling frequency if internal filter is disabled
        ack = mpuConfiguration.write(MPU_RA_SMPLRT_DIV, 0x07);  //SMPLRT_DIV    -- SMPLRT_DIV = 7  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    else
        ack = mpuConfiguration.write(MPU_RA_SMPLRT_DIV, 0x00);
    ack = mpuConfiguration.write(MPU_RA_CONFIG, mpuLowPassFilter); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    ack = mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);   //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

    // ACC Init stuff.
    // Accel scale 8g (4096 LSB/g)
    ack = mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);

    ack = mpuConfiguration.write(MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS

#ifdef USE_MPU_DATA_READY_SIGNAL
    ack = mpuConfiguration.write(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif
#ifdef ACCGYRO_FIFO
    mpu6050FifoEnable();
#endif
    UNUSED(ack);
}

int mpu6050GetFifoLen(void)
{
    uint8_t buf[2];
    mpuConfiguration.read(MPU_RA_FIFO_COUNTH, 2, buf);
    return (buf[0] << 8) | buf[1];
}

void mpu6050FifoEnable(void)
{
    mpuConfiguration.write(MPU_RA_USER_CTRL, MPU_RF_FIFO_RESET);   // flush FIFO
    mpuConfiguration.write(MPU_RA_USER_CTRL, MPU_RF_FIFO_EN);      // enable FIFO
    mpuConfiguration.write(MPU_RA_FIFO_EN, MPU_RF_XG_FIFO_EN | MPU_RF_YG_FIFO_EN | MPU_RF_ZG_FIFO_EN | MPU_RF_ACCEL_FIFO_EN);
}

void mpu6050FifoFlush(void)
{
    mpuConfiguration.write(MPU_RA_USER_CTRL, MPU_RF_FIFO_RESET);   // flush FIFO
    mpuConfiguration.write(MPU_RA_USER_CTRL, MPU_RF_FIFO_EN);      // enable FIFO
}

int mpu6050FifoRead(uint8_t *buffer, int maxLen, int modulo) {
    int fifoLen;
    int tries=2;
    do {
        fifoLen = mpu6050GetFifoLen();
        // read length again if only partial data are in FIFO (rest of registers will be writen very soon)
        // this ensures that out-of-sync condition is easily detected
        pinDbgToggle(DBP_MPU6050_1);
    } while(--tries && (fifoLen < maxLen && fifoLen % modulo));
    int len = MIN(maxLen, fifoLen);
    if(len>12 && len <= 36)
        pinDbgToggle(DBP_MPU6050_2);
    if(len)
        if(!mpuConfiguration.read(MPU_RA_FIFO_R_W, len, buffer))
            return -1;
    return len;
}

// return number of samples processed
int mpu6050GyroAccFetch(void)
{
    int16_t gyroAccBuffer[8][6];   // TODO - uint8_t limitation for size
    int len = mpu6050FifoRead(((uint8_t*)gyroAccBuffer), sizeof(gyroAccBuffer), 12);
    int idx = 0;
    while(len > 0) {
        if(len < 12) {
            mpu6050FifoFlush();
            return -1;
        }
        for(int i = 0; i < 6; i++)
            gyroAccBuffer[idx][i] = __builtin_bswap16(gyroAccBuffer[idx][i]);
        accHandleData(&gyroAccBuffer[idx][0]);
        gyroHandleData(&gyroAccBuffer[idx][3]);
        len -= 12; idx++;
    }
    return idx;
}
