
#pragma once

/*
  Deferred interrupt call mechanism. pendSV interrupt is used to call processing routine on lower priority.
  Usefull to do keep interrupt routines extremely short (and use queues to pass data) and do
  the time-consuming work at the end of ISR. This mechanism could be easily replaced with dedicated threads
  if(when) RTOS is used.

  This implementation supports only limited number of callable routines. Each routine has one bit used to trigger call.
*/

#include <stdint.h>

typedef struct callbackRec_s callbackRec_t;
typedef void callbackFun_t(callbackRec_t* data);
struct callbackRec_s {
    callbackFun_t* fn;
    uint8_t id;
};

void callbackInit(void);
void callbackRegister(callbackRec_t *self, callbackFun_t *fn);
void callbackRelease(callbackRec_t *self);
void callbackTrigger(callbackRec_t *self);
