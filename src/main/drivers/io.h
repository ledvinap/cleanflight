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

int IO_GPIOPortIdx(const ioDef_t *io);
int IO_GPIOPortSource(const ioDef_t *io);
int IO_GPIOPinIdx(const ioDef_t *io);
int IO_GPIOPinSource(const ioDef_t *io);
uint32_t IO_EXTILine(const ioDef_t *io);

bool IODigitalRead(const ioDef_t *  io);
void IODigitalWrite(const ioDef_t *  io, bool value);
void IOConfigGPIO(const ioDef_t *  io, GPIO_Mode mode);
void IOConfigGPIOAF(const ioDef_t *  io, GPIO_Mode mode, uint8_t af);
