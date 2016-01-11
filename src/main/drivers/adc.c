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

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "system.h"

#include "adc.h"
#include "adc_impl.h"

//#define DEBUG_ADC_CHANNELS

#ifdef USE_ADC
adc_config_t adcConfig[ADC_CHANNEL_COUNT];
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

// prepare ADC channel
bool adcInitChannel(AdcChannel channel, ioTag_t ioTag)
{
    if(!ioTag || !adcHwIOAvailable(IOGetByTag(ioTag)))
        return false;
    adcConfig[channel].pin = ioTag;
    return true;
}

void adcInit(drv_adc_config_t *init)
{
    adcInitHw();
#ifdef VBAT_ADC_IO
    if (init->enableVBat) {
        adcInitChannel(ADC_BATTERY, IO_TAG(VBAT_ADC_IO));
    }
#endif
#ifdef RSSI_ADC_IO
    if (init->enableRSSI) {
        adcInitChannel(ADC_RSSI, IO_TAG(RSSI_ADC_IO));
    }
#endif
#ifdef EXTERNAL1_ADC_IO
    if (init->enableExternal1) {
        adcInitChannel(ADC_EXTERNAL1, IO_TAG(EXTERNAL1_ADC_IO));
    }
#endif
#ifdef CURRENT_METER_ADC_IO
    if (init->enableCurrentMeter) {
        adcInitChannel(ADC_CURRENT, IO_TAG(CURRENT_METER_ADC_IO));
    }
#endif
    adcHwStart();
}

uint16_t adcGetChannel(uint8_t channel)
{
#ifdef DEBUG_ADC_CHANNELS
    if (adcConfig[0].pin) {
        debug[0] = adcValues[adcConfig[0].dmaIndex];
    }
    if (adcConfig[1].pin) {
        debug[1] = adcValues[adcConfig[1].dmaIndex];
    }
    if (adcConfig[2].pin) {
        debug[2] = adcValues[adcConfig[2].dmaIndex];
    }
    if (adcConfig[3].pin) {
        debug[3] = adcValues[adcConfig[3].dmaIndex];
    }
#endif
    return adcValues[adcConfig[channel].dmaIndex];
}

#else
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}
#endif
