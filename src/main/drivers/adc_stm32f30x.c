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
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "system.h"

#include "drivers/rcc.h"

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

#define ADC_IOPIN_COUNT 16
#define ADC_COUNT 4

#define DEFM(c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16) \
    { IO_TAG_E(c1), IO_TAG_E(c2),  IO_TAG_E(c3),  IO_TAG_E(c4),  IO_TAG_E(c5),  IO_TAG_E(c6),  IO_TAG_E(c7),  IO_TAG_E(c8), \
      IO_TAG_E(c9), IO_TAG_E(c10), IO_TAG_E(c11), IO_TAG_E(c12), IO_TAG_E(c13), IO_TAG_E(c14), IO_TAG_E(c15), IO_TAG_E(c16)}

//                                           1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16
ioTag_t adcMap[ADC_COUNT][ADC_IOPIN_COUNT] = {
    DEFM(PA0,  PA1,  PA2,  PA3,  PF4,  PC0,  PC1,  PC2,  PC3,  PF2,  NONE, NONE, NONE, NONE, NONE, NONE),
    DEFM(PA4,  PA5,  PA6,  PA7,  PC4,  PC0,  PC1,  PC2,  PC3,  PF2,  PC5,  PB2,  NONE, NONE, NONE, NONE),
    DEFM(PB1,  PE9,  PE13, NONE, PB13, PE8,  PD10, PD11, PD12, PD13, PD14, PB0,  PE7,  PE10, PE11, PE12),
    DEFM(PE14, PE15, PB12, PB14, PB15, PE8,  PD10, PD11, PD12, PD13, PD14, PD8,  PD9,  NONE, NONE, NONE),
};

#undef DEFM

typedef struct adcDef_s {
    ADC_TypeDef *adc;
    rccPeriphTag_t rcc;
    uint8_t irq;
} adcDef_t;

adcDef_t adcDef[ADC_COUNT] = {
    {ADC1, RCC_AHB(ADC12), ADC1_2_IRQn},
    {ADC2, RCC_AHB(ADC12), ADC1_2_IRQn},
    {ADC3, RCC_AHB(ADC34), ADC3_IRQn},
    {ADC4, RCC_AHB(ADC34), ADC4_IRQn},
};

int8_t channelCount[ADC_COUNT];

void adcInitHw(void)
{
    memset(&adcConfig, 0, sizeof(adcConfig));
}

// return adc mask in upper byte, channel index in lower  byte
// returns zero (no ADC in upper byte) when not found (different from 103 implementation)
uint16_t adcHwIOChannel(IO_t io)
{
    uint16_t ret;
    if(!io) return -1;
    for(unsigned adc = 0; adc < ADC_COUNT; adc++)
        for(int i = 0; i < ADC_IOPIN_COUNT; i++)
            if(IOGetByTag(adcMap[adc][i]) == io) {
                ret = 0x100 << adc;                         // bit for first ADC
#if 0           // first ADC only, TODO better channel mapping
                if((adc % 2) == 0                           // lower channel
                   && adc +1 < ADC_COUNT                    // next ADC exists
                   && IOGetByTag(adcMap[adc + 1][i]) == io) // same IO
                    ret |= 0x100 << (adc + 1);              // bit for second ADC
#endif
                return ret | i;
            }
    return 0;
}

// check if pin is available on ADC
bool adcHwIOAvailable(IO_t io)
{
    return adcHwIOChannel(io) != 0;
}


void adcHwStart(void)
{
    // scan all channels, configure pins as input
    int channelCount[ADC_COUNT];
    memset(channelCount, 0, sizeof(channelCount));
    for(int i = 0; i < (int)ARRAYLEN(adcConfig); i++) {
        IO_t io = IOGetByTag(adcConfig[i].pin);
        if(!io || !adcHwIOAvailable(io))
            continue;
        IOInit(io, OWNER_SYSTEM, RESOURCE_ADC | RESOURCE_INPUT);
        IOConfigGPIO(io, IOCFG_ANALOG);
        // sampletime is hardwired now
        adcConfig[i].sampleTime = ADC_SampleTime_601Cycles5;
        #warning "HANDLE ADC"
        //channelCount[adc]++;
    }

    // enable ADCs (all are enabled now, TODO)
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8);  // 9MHz from 72MHz APB2 clock(HSE)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

    RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div8);
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_ADC34, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    unsigned memoryIndex = 0; // index in results memory

    for(int adc = 0; adc < ADC_COUNT; adc++) {
        if(!channelCount[adc])
            continue;

        ADC_CommonInitTypeDef ADC_CommonInitStructure;
        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;     // TODO - unify with 103
        ADC_CommonInit(adcDef[adc].adc, &ADC_CommonInitStructure);

        ADC_InitTypeDef ADC_InitStructure;
        ADC_StructInit(&ADC_InitStructure);
        ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Disable;
        ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
        ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfRegChannel       = channelCount[adc];
        ADC_Init(adcDef[adc].adc, &ADC_InitStructure);

        int sequencerIndex = 1;
        for(int i = 0; i < (int)ARRAYLEN(adcConfig); i++) {
            if (!(adcHwIOChannel(IOGetByTag(adcConfig[i].pin)) & (0x100 << adc))) {
                continue; // not on this ADC
            }
            adcConfig[i].dmaIndex = memoryIndex++;
            ADC_RegularChannelConfig(adcDef[adc].adc, adcConfig[i].adcChannel & 0xff, sequencerIndex++, adcConfig[i].sampleTime);
        }

        // calibrate

        ADC_VoltageRegulatorCmd(adcDef[adc].adc, ENABLE);
        delay(10);
        ADC_SelectCalibrationMode(adcDef[adc].adc, ADC_CalibrationMode_Single);
        ADC_StartCalibration(adcDef[adc].adc);
        while(ADC_GetCalibrationStatus(adcDef[adc].adc) != RESET);
        ADC_VoltageRegulatorCmd(adcDef[adc].adc, DISABLE);
        ADC_Cmd(adcDef[adc].adc, ENABLE);
        while(!ADC_GetFlagStatus(adcDef[adc].adc, ADC_FLAG_RDY));
    }
    // enable 

}

// start conversion scan on all channels. Called from systick handler now
void adcTriggerPeriodicConversion(void)
{
    for(int adc = 0; adc < ADC_COUNT; adc++) {
        if(!channelCount[adc])
            continue;
        ADC_StartConversion(adcDef[adc].adc);
    }
}
