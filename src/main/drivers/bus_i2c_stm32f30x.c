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

#include <platform.h>

#include "build_config.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "system.h"

#include "bus_i2c.h"

#ifndef SOFT_I2C

typedef struct i2cHwDef_s {
    I2C_TypeDef *i2cDev;
    const ioDef_t *sclIO, *sdaIO;
    uint32_t rccI2CCLKConfig;
    rccPeriphTag_t rcc;
    uint8_t afConfig;
} i2cHwDef_t;

#ifndef I2C1_SCL_IO
# define I2C1_SCL_IO &IO_PB6
# define I2C1_SDA_IO &IO_PB7
# define I2C1_AF GPIO_AF_4
#endif

struct i2cHwDef_s i2c1Def = {
    .i2cDev = I2C1,
    .sclIO = I2C1_SCL_IO,
    .sdaIO = I2C1_SDA_IO,
    .rccI2CCLKConfig = RCC_I2C1CLK_SYSCLK,
    .rcc = RCC_APB1(I2C1),
    .afConfig = I2C1_AF,
};

#ifndef I2C2_SCL_IO
# define I2C2_SCL_IO &IO_PF6
# define I2C2_SDA_IO &IO_PA10
# define I2C2_AF GPIO_AF_4
#endif

struct i2cHwDef_s i2c2Def = {
    .i2cDev = I2C2,
    .sclIO = I2C2_SCL_IO,   // this pin is available only on TQFP100 package
    .sdaIO = I2C2_SDA_IO,
    .rccI2CCLKConfig = RCC_I2C2CLK_SYSCLK,
    .rcc = RCC_APB1(I2C2),
    .afConfig = I2C2_AF,
};


#define I2C_SHORT_TIMEOUT             ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * I2C_SHORT_TIMEOUT))

static uint32_t i2cTimeout;

static volatile uint16_t i2c1ErrorCount = 0;
static volatile uint16_t i2c2ErrorCount = 0;

static I2C_TypeDef *I2Cx = NULL;

///////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

uint32_t i2cTimeoutUserCallback(I2C_TypeDef *I2Cx)
{
    if (I2Cx == I2C1) {
        i2c1ErrorCount++;
    } else {
        i2c2ErrorCount++;
    }
    return false;
}

void i2cInitPort(const struct i2cHwDef_s* def)
{

    RCC_ClockCmd(def->rcc, ENABLE);
    RCC_I2CCLKConfig(def->rccI2CCLKConfig);

    //i2cUnstick(I2Cx);                                         // Clock out stuff to make sure slaves arent stuck

    IOConfigGPIOAF(def->sclIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), def->afConfig);
    IOConfigGPIOAF(def->sdaIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), def->afConfig);

    I2C_InitTypeDef I2C_InitStructure = {
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_AnalogFilter = I2C_AnalogFilter_Enable,
        .I2C_DigitalFilter = 0x00,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
        .I2C_Timing = 0x00E0257A, // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.
        //.I2C_Timing              = 0x8000050B;
    };
    I2C_Init(def->i2cDev, &I2C_InitStructure);

    I2C_Cmd(def->i2cDev, ENABLE);


    // FIXME timing is board specific - I2C2 :
    //   I2C_InitStructure.I2C_Timing = 0x00310309; // //400kHz I2C @ 8MHz input -> PRESC=0x0, SCLDEL=0x3, SDADEL=0x1, SCLH=0x03, SCLL=0x09 - value from TauLabs/Sparky
    //    ^ when using this setting and after a few seconds of a scope probe being attached to the I2C bus it was observed that the bus enters
    //    a busy state and does not recover.
}

void i2cInit(I2CDevice index)
{
    if (index == I2CDEV_1) {
        I2Cx = I2C1;
        i2cInitPort(&i2c1Def);
    } else {
        I2Cx = I2C2;
        i2cInitPort(&i2c2Def);
    }
}

uint16_t i2cGetErrorCounter(void)
{
    if (I2Cx == I2C1) {
        return i2c1ErrorCount;
    }

    return i2c2ErrorCount;

}

bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data)
{
    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TCR flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Write data to TXDR */
    I2C_SendData(I2Cx, data);

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return true;
}

bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TC flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while (len) {
        /* Wait until RXNE flag is set */
        i2cTimeout = I2C_LONG_TIMEOUT;
        while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return i2cTimeoutUserCallback(I2Cx);
            }
        }

        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2Cx);
        /* Point to the next location where the byte read will be saved */
        buf++;

        /* Decrement the read bytes counter */
        len--;
    }

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}

#endif
