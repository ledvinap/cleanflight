#pragma once

// TODO - GPIO_TypeDef include
#include "gpio.h"
#include "io.h"

typedef struct ioDef_s ioDef_t;
struct ioDef_s {
    ioTag_t tag;
};

struct ioRec_s {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    resourceOwner_t owner;
    resourceType_t resourcesUsed; // TODO!
};

#define DEFIO_IO_REC_INITIALIZER {NULL, 0, 0, 0}

int IO_GPIOPortIdx(ioRec_t *io);
int IO_GPIOPinIdx(ioRec_t *io);

int IO_GPIO_PinSource(ioRec_t *io);
int IO_EXTI_PortSourceGPIO(ioRec_t *io);
int IO_EXTI_PinSource(ioRec_t *io);
uint32_t IO_EXTI_Line(ioRec_t *io);
