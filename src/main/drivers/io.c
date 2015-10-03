
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "target_io.h"
#include "target.h"

// io ports defs are stored in array by index now
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

// port index, GPIOA == 0
int IO_GPIOPortIdx(ioRec_t *io)
{
    if(!io)
        return -1;
    return (((size_t)io->gpio - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_EXTI_PortSourceGPIO(ioRec_t *io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(ioRec_t *io)
{
    if(!io)
        return -1;
    return 31 - __builtin_clz(io->pin);  // CLZ is a bit faster than FFS
}

int IO_EXTI_PinSource(ioRec_t *io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PinSource(ioRec_t *io)
{
    return IO_GPIOPinIdx(io);
}

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(ioRec_t *io)
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

bool IODigitalRead(ioRec_t *io)
{
    if(!io)
        return false;

    return digitalIn(io->gpio, io->pin);
}

void IODigitalWrite(ioRec_t *io, bool value)
{
    if(!io)
        return;
    if(value)
        digitalHi(io->gpio, io->pin);
    else
        digitalLo(io->gpio, io->pin);
}

// claim IO pin, set owner and resources
void IOInit(ioRec_t *io, resourceOwner_t owner, resourceType_t resources)
{
    if(owner != OWNER_FREE)   // pass OWNER_FREE to keep old owner
        io->owner = owner;
    io->resourcesUsed |= resources;
}

// release IO pin
void IORelease(ioRec_t *io)
{
    io->owner = OWNER_FREE;
}

resourceOwner_t IOGetOwner(ioRec_t *io)
{
    return io->owner;
}

resourceType_t IOGetResources(ioRec_t *io)
{
    return io->resourcesUsed;
}

#if defined(STM32F10X)

void IOConfigGPIO(ioRec_t *io, ioConfig_t cfg)
{
    if(!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = io->pin,
        .GPIO_Speed = cfg & 0x03,
        .GPIO_Mode =  cfg & 0x7c,
    };
    GPIO_Init(io->gpio, &init);
}


#elif defined(STM32F303xC)

void IOConfigGPIO(ioRec_t *io, ioConfig_t cfg)
{
    if(!io)
        return;
    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = io->pin,
        .GPIO_Mode =  (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(io->gpio, &init);
}

void IOConfigGPIOAF(ioRec_t *io, ioConfig_t cfg, uint8_t af)
{
    if(!io)
       return;

    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    GPIO_PinAFConfig(io->gpio, IO_GPIO_PinSource(io), af);

    GPIO_InitTypeDef init = {
        .GPIO_Pin   = io->pin,
        .GPIO_Mode  = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd  = (cfg >> 5) & 0x03,
    };
    GPIO_Init(io->gpio, &init);
}
#endif

// IO subsystem initialization
// emit ioRec_t
DEFIO_IO_DEFINE();
// generate mask for used port pins
DEFIO_IO_DEFINE_MASK();


extern const ioDef_t _tab_io_def_start, _tab_io_def_end;
extern ioRec_t _tab_io_rec_start, _tab_io_rec_end;
// initialize all ioRec_t structures from ROM
void IOInitGlobal(void) {
    ioRec_t *io = &_tab_io_rec_start;

    for(unsigned port = 0; port < ARRAYLEN(ioDefMask); port++)
        for(unsigned pin = 0; pin < sizeof(ioDefMask[0]) * 8; pin++)
            if(ioDefMask[port] & (1 << pin)) {
                io->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                io->pin = 1 << pin;
                io++;
            }
    if(io != &_tab_io_rec_end)
    {}; // TODO - abort
}

ioRec_t* IOGetByTag(ioTag_t tag)
{
    for(ioRec_t *io = &_tab_io_rec_start; io < &_tab_io_rec_end; io++)
        if(IO_GPIOPortIdx(io) == DEFIO_IO_TAG_GPIOID(tag)
           && IO_GPIOPinIdx(io) == DEFIO_IO_TAG_PIN(tag))
            return io;
    return NULL;
}
