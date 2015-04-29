
#include "common/utils.h"

#include "drivers/io.h"

#include "target_io.h"
#include "target.h"

int IO_GPIOPortIdx(const ioDef_t *io)
{
    if(!io)
        return -1;
    return (((size_t)io->gpio - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_GPIOPortSource(const ioDef_t *io)
{
    return IO_GPIOPortIdx(io);
}

int IO_GPIOPinIdx(const ioDef_t *io)
{
    if(!io)
        return -1;
    return 31 - __builtin_clz(io->pin);  // CLZ is a bit faster than FFS
}

int IO_GPIOPinSource(const ioDef_t *io)
{
    return IO_GPIOPinIdx(io);
}

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTILine(const ioDef_t *io)
{
    if(!io)
        return 0;
#if defined(STM32F10X)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(STM32F30X)
    return IO_GPIOPinIdx(io);
#endif
}

bool IODigitalRead(const ioDef_t *io)
{
    if(!io)
        return false;

    return digitalIn(io->gpio, io->pin);
}

void IODigitalWrite(const ioDef_t *io, bool value)
{
    if(!io)
        return;
    if(value)
        digitalHi(io->gpio, io->pin);
    else
        digitalLo(io->gpio, io->pin);
}


void IOConfigGPIO(const ioDef_t *io, GPIO_Mode mode)
{
    if(!io)
        return;

    gpio_config_t cfg;

    cfg.pin = io->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(io->gpio, &cfg);
}
