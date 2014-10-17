
#pragma once

#define TIMEROUT_QUEUE_LEN (1<<5)
#define TIMEROUT_QUEUE_LOW 5

#define TIMEROUT_RUNNING       0x0100
#define TIMEROUT_INVERTED      0x0200          // output is inverted
#define TIMEROUT_WAKEONEMPTY   0x0400          // wake caller on last interval end
#define TIMEROUT_WAKEONLOW     0x0800          // wake caller on last interval end
#define TIMEROUT_RESTART       0x1000          // interval was inserted in last period, restart transmission in IRQ


typedef struct timerOutputRec {
    uint16_t queue[TIMEROUT_QUEUE_LEN];
    unsigned qhead;                             // queue head used for sending data
    unsigned qheadUnc;                          // index of uncommitted data head
    unsigned qtail;
    unsigned qtailWake;                         // queue index used to wake caller (compared after fetch in irq)
    uint16_t flags;
    const timerHardware_t* timHw;
    TIM_TypeDef *tim;
    volatile timCCR_t* timCCR;
    callbackRec_t *callback;
    timerCCHandlerRec_t compareCb;
} timerOutputRec_t;

void timerOut_Config(timerOutputRec_t *self, const timerHardware_t *timHw, channelType_t owner, int priority, callbackRec_t *callback, uint16_t tim_OCPolarity);
void timerOut_Release(timerOutputRec_t *self);
void timerOut_Restart(timerOutputRec_t *self);
short timerOut_QSpace(timerOutputRec_t *self);
short timerOut_QLen(timerOutputRec_t *self);
bool timerOut_QPush(timerOutputRec_t *self, uint16_t delta, uint16_t flags);
void timerOut_QCommit(timerOutputRec_t *self);
