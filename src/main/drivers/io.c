
#include "common/utils.h"

#include "target_io.h"
#include "target.h"
#include "timer.h"


const timerHardware_t *getIOHw(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx < USABLE_IO_CHANNEL_COUNT) return &timerHardware[idx];  // TODO!!!
    else return NULL;
}

int IO_GPIOPortIdx(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx >= USABLE_IO_CHANNEL_COUNT) return -1;
    return (((size_t)timerHardware[idx].gpio - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_GPIOPortSource(IOId_t id)
{
    return IO_GPIOPortIdx(id);
}

int IO_GPIOPinIdx(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx >= USABLE_IO_CHANNEL_COUNT) return -1;
    return 31 - __builtin_clz(timerHardware[idx].pin);  // CLZ is a bit faster than FFS
}

int IO_GPIOPinSource(IOId_t id)
{
    return IO_GPIOPinIdx(id);
}

// mask on stm32f103, bit index on stm32f303
int IO_EXTILine(IOId_t id)
{
#if defined(STM32F10X)
    return 1 << IO_GPIOPinIdx(id);
#elif defined(STM32F30X)
    return IO_GPIOPinIdx;
#endif
}


bool IO_DigitalRead(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx >= USABLE_IO_CHANNEL_COUNT)
        return false;

    return digitalIn(timerHardware[idx].gpio, timerHardware[idx].pin);
}

void IO_DigitalWrite(IOId_t id, bool value)
{
    unsigned idx = id - IO1;
    if(idx >= USABLE_IO_CHANNEL_COUNT)
        return;
    if(value)
        digitalHi(timerHardware[idx].gpio, timerHardware[idx].pin);
    else
        digitalLo(timerHardware[idx].gpio, timerHardware[idx].pin);
}


void IO_ConfigGPIO(IOId_t id, GPIO_Mode mode)
{
    unsigned idx = id - IO1;
    if(idx >= USABLE_IO_CHANNEL_COUNT)
        return;

    gpio_config_t cfg;

    cfg.pin = timerHardware[idx].pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(timerHardware[idx].gpio, &cfg);
}
