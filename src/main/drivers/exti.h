#pragma once

#include <stdbool.h>

#include "drivers/io.h"

typedef struct extiCallbackRec_s extiCallbackRec_t;
typedef void extiHandlerCallback(extiCallbackRec_t *self);

struct extiCallbackRec_s {
    extiHandlerCallback *fn;
};

void EXTIInit(void);

void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn);
void EXTIConfig(ioRec_t *ioRec, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger);
void EXTIEnable(ioRec_t *ioRec, bool enable);
