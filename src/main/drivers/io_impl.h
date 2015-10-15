#pragma once

// TODO - GPIO_TypeDef include
#include "gpio.h"
#include "io.h"
#include "platform.h"

typedef struct ioDef_s {
    ioTag_t tag;
} ioDef_t;

typedef struct ioRec_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    resourceOwner_t owner;
    resourceType_t resourcesUsed; // TODO!
} ioRec_t;

#define DEFIO_IO_REC_INITIALIZER {NULL, 0, 0, 0}

extern ioRec_t ioRecs[DEFIO_IO_USED_COUNT];

int IO_GPIOPortIdx(ioRec_t *io);
int IO_GPIOPinIdx(ioRec_t *io);
#if defined(STM32F10X)
int IO_GPIO_PinSource(ioRec_t *io);
int IO_GPIO_PortSource(ioRec_t *io);
#elif defined(STM32F303xC)
int IO_EXTI_PortSourceGPIO(ioRec_t *io);
int IO_EXTI_PinSource(ioRec_t *io);
#endif
uint32_t IO_EXTI_Line(ioRec_t *io);
