#pragma once

#include <stdbool.h>

#include "target_io.h"
#include "timer.h"

const timerHardware_t* getIOHw(IOId_t id);

int IO_GPIOPortIdx(IOId_t id);
int IO_GPIOPortSource(IOId_t id);
int IO_GPIOPinIdx(IOId_t id);
int IO_GPIOPinSource(IOId_t id);
uint32_t IO_EXTILine(IOId_t id);

bool IO_DigitalRead(IOId_t id);
void IO_DigitalWrite(IOId_t id, bool value);
void IO_ConfigGPIO(IOId_t id, GPIO_Mode mode);
