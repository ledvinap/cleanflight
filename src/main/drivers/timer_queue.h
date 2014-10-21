 #pragma once

extern const timerHardware_t timerQueueHardware;  // TODO

#define TIMERQUEUE_FLAG_QUEUED 1       // timer is in timer queue 
#define TIMERQUEUE_FLAG_ISR    2       // timer is in fast part of timer heap

struct timerQueueRec_s;
typedef void timerQueueCallbackFn(struct timerQueueRec_s *self);

typedef struct timerQueueRec_s {
    uint16_t time;
    uint16_t timeISR;
    uint8_t flags;
    uint8_t inIsrQueue;
    timerQueueCallbackFn *callbackFn;
} timerQueueRec_t;

void timerQueue_Init(void);
void timerQueue_Config(timerQueueRec_t *self, timerQueueCallbackFn *callbackFn);
void timerQueue_Release(timerQueueRec_t *self);
void timerQueue_Start(timerQueueRec_t *self, int16_t timeout);
bool timerQueue_IsRunning(timerQueueRec_t *self, int16_t timeout);
