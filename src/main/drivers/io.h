#pragma once

#include <stdbool.h>

#include "drivers/gpio.h"
#include "drivers/resource.h"

#ifdef STM32F10X
# include "drivers/io_stm32f10x.h"
#endif

#ifdef STM32F303xC
# include "drivers/io_stm32f30x.h"
#endif

typedef struct ioDef_s {
    GPIO_TypeDef *gpio;
    uint32_t pin;
    struct ioRec_s *rec;
} ioDef_t;

typedef struct ioRec_s {
    resourceOwner_t owner;
    resourceType_t resourcesUsed; // TODO!
} ioRec_t;

typedef uint8_t ioConfig_t;

#if defined(STM32F10X)

// mode is using bits 6-2
# define IO_CONFIG(mode, speed) ((mode) | (speed))

# define IOCFG_AF_PP IO_CONFIG(GPIO_Mode_AF_PP, GPIO_Speed_2MHz)
# define IOCFG_IPD IO_CONFIG(GPIO_Mode_IPD, GPIO_Speed_2MHz)
# define IOCFG_IPU IO_CONFIG(GPIO_Mode_IPU, GPIO_Speed_2MHz)

#elif defined(STM32F303xC)

// define it here to be compatible with STM32F10X
#define GPIO_Speed_10MHz GPIO_Speed_Level_1  // Fast Speed:10MHz
#define GPIO_Speed_2MHz  GPIO_Speed_Level_2  // Medium Speed:2MHz (same as zero)
#define GPIO_Speed_50MHz GPIO_Speed_Level_3  // High Speed:50MHz

# define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

# define IOCFG_AF_PP IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
# define IOCFG_IPD IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_DOWN)
# define IOCFG_IPU IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP)

#endif

int IO_GPIOPortIdx(const ioDef_t *io);
int IO_GPIOPortSource(const ioDef_t *io);
int IO_GPIOPinIdx(const ioDef_t *io);
int IO_GPIOPinSource(const ioDef_t *io);
uint32_t IO_EXTILine(const ioDef_t *io);

bool IODigitalRead(const ioDef_t *io);
void IODigitalWrite(const ioDef_t *io, bool value);

void IOInit(const ioDef_t *io, resourceOwner_t owner, resourceType_t resources);
void IORelease(const ioDef_t *io);
resourceOwner_t IOGetOwner(const ioDef_t *io);

void IOConfigGPIO(const ioDef_t *io, ioConfig_t cfg);
#if defined(STM32F303xC)
void IOConfigGPIOAF(const ioDef_t *io, ioConfig_t cfg, uint8_t af);
#endif
