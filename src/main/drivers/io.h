#pragma once

#include <stdbool.h>

#include "drivers/gpio.h"
#include "drivers/resource.h"

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
