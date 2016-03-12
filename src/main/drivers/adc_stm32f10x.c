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

#include <platform.h>

#include "build_config.h"

#include "system.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "drivers/rcc.h"

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"


#define ADC_IOPIN_COUNT 16

#define DEFM(c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16)    \
    { IO_TAG_E(c1), IO_TAG_E(c2),  IO_TAG_E(c3),  IO_TAG_E(c4),  IO_TAG_E(c5),  IO_TAG_E(c6),  IO_TAG_E(c7),  IO_TAG_E(c8), \
      IO_TAG_E(c9), IO_TAG_E(c10), IO_TAG_E(c11), IO_TAG_E(c12), IO_TAG_E(c13), IO_TAG_E(c14), IO_TAG_E(c15), IO_TAG_E(c16)}

ioTag_t adc12Map[ADC_IOPIN_COUNT] =
    DEFM(PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5);
#undef DEFM

// ADC3 is available on high-density devices in 144pin package. It is not supported now

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#define ADC_RCC                     RCC_APB2(ADC1)
#define ADC_DMA_RCC                 RCC_AHB(DMA1)
#define ADC_DMA_CHANNEL             DMA1_Channel1
#endif


// Driver for STM32F103CB onboard ADC
//
// Naze32
// Battery Voltage (VBAT) is connected to PA4 (ADC1_IN4) with 10k:1k divider
// RSSI ADC uses CH2 (PA1, ADC1_IN1)
// Current ADC uses CH8 (PB1, ADC1_IN9)
//
// NAZE rev.5 hardware has PA5 (ADC1_IN5) on breakout pad on bottom of board
//


void adcInitHw(void)
{
    memset(&adcConfig, 0, sizeof(adcConfig));
}


// return channel if pin is ADC input, -1 otherwise
// it is assumed that calling code is not time-critical
int adcHwIOChannel(IO_t io)
{
    if(!io) return -1;
    for(int i = 0; i < (int)ARRAYLEN(adc12Map); i++)
        if(IOGetByTag(adc12Map[i]) == io)
            return i;
    return -1;
}

// check if pin is available on ADC
bool adcHwIOAvailable(IO_t io)
{
    return adcHwIOChannel(io) >= 0;
}

void adcHwStart(void)
{
    // scan all channels, configure pins as input
    int channelCount = 0;
    for(int i = 0; i < (int)ARRAYLEN(adcConfig); i++) {
        IO_t io = IOGetByTag(adcConfig[i].pin);
        if(!io || !adcHwIOAvailable(io))
            continue;
        IOInit(io, OWNER_SYSTEM, RESOURCE_ADC | RESOURCE_INPUT);
        IOConfigGPIO(io, IOCFG_ANALOG);
        // sampletime is hardwired now
        adcConfig[i].sampleTime = ADC_SampleTime_239Cycles5;
        channelCount++;
    }
    if(!channelCount)
        return;

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // 9MHz from 72MHz APB2 clock(HSE), 8MHz from 64MHz (HSI)
    RCC_ClockCmd(ADC_RCC, ENABLE);
    RCC_ClockCmd(ADC_DMA_RCC, ENABLE);

    DMA_DeInit(ADC_DMA_CHANNEL);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_INSTANCE->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = channelCount;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = channelCount > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);
    DMA_Cmd(ADC_DMA_CHANNEL, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = channelCount > 1 ? ENABLE : DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = channelCount;
    ADC_Init(ADC_INSTANCE, &ADC_InitStructure);

    int sequencerIndex = 0;
    for (int i = 0; i < (int)ARRAYLEN(adcConfig); i++) {
        if (!adcConfig[i].pin) {
            continue;
        }
        adcConfig[i].dmaIndex = sequencerIndex;
        ADC_RegularChannelConfig(ADC_INSTANCE, adcHwIOChannel(IOGetByTag(adcConfig[i].pin)), sequencerIndex + 1, adcConfig[i].sampleTime);
        sequencerIndex++;
    }

    ADC_DMACmd(ADC_INSTANCE, ENABLE);
    ADC_Cmd(ADC_INSTANCE, ENABLE);

    ADC_ResetCalibration(ADC_INSTANCE);
    while(ADC_GetResetCalibrationStatus(ADC_INSTANCE));
    ADC_StartCalibration(ADC_INSTANCE);
    while(ADC_GetCalibrationStatus(ADC_INSTANCE));

    ADC_SoftwareStartConvCmd(ADC_INSTANCE, ENABLE);
}

// start conversion scan on all channels. Called from systick handler now
void adcTriggerPeriodicConversion(void)
{
    ADC_SoftwareStartConvCmd(ADC_INSTANCE, ENABLE);
}
