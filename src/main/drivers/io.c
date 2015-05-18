
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"

#include "target_io.h"
#include "target.h"

// io ports defs are stored in array by index now
// TODO - tag them by GPIO addr (or index) and search for them when necesary ... 
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(STM32F10X)
const struct ioPortDef_s ioPortDefs[] = {
    {RCC_APB2(IOPA)},
    {RCC_APB2(IOPB)},
    {RCC_APB2(IOPC)},
    {RCC_APB2(IOPD)},
    {RCC_APB2(IOPE)},
    {
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
        RCC_APB2(IOPF),
#else
        0,
#endif
    },
    {
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
        RCC_APB2(IOPG),
#else
        0,
#endif
    },
};
#elif defined(STM32F303xC)
const struct ioPortDef_s ioPortDefs[] = {
    {RCC_AHB(GPIOA)},
    {RCC_AHB(GPIOB)},
    {RCC_AHB(GPIOC)},
    {RCC_AHB(GPIOD)},
    {RCC_AHB(GPIOE)},
    {RCC_AHB(GPIOF)},
};
#endif

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
#elif defined(STM32F303xC)
    return IO_GPIOPinIdx(io);
#else
# error "Unknown target type"
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
    // TODO!
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    gpio_config_t cfg;

    cfg.pin = io->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(io->gpio, &cfg);
}

#ifdef STM32F303xC

void IOConfigGPIOAF(const ioDef_t *io, GPIO_Mode mode, uint8_t af)
{
    if(!io)
       return;

    // TODO!

    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_PinAFConfig(io->gpio, IO_GPIOPinSource(io), af);

    gpio_config_t cfg;

    cfg.pin = io->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(io->gpio, &cfg);
}

#endif
