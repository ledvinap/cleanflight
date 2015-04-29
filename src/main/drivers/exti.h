#pragma once

#include <stdbool.h>

#include "drivers/io.h"

struct extiCallbackRec_s;
typedef void extiHandlerCallback(struct extiCallbackRec_s *self);

typedef struct extiCallbackRec_s {
    extiHandlerCallback *fn;
} extiCallbackRec_t;

void EXTIInit(void);

void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn);
void EXTIConfig(const ioDef_t *ioDef, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger);
void EXTIEnable(const ioDef_t *ioDef, bool enable);
