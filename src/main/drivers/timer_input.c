#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "callback.h"
#include "system.h"
#include "nvic.h"

#include "timer.h"
#include "timer_queue.h"
#include "timer_input.h"

/* timer input fast handler. Input signal generates capture events,
   captured times are augmented with input state and stored in receive queue
*/
void timerIn_timerCaptureEvent(timerCCHandlerRec_t *self_, uint16_t capture);
void timerIn_dualCaptureEventStart(timerCCHandlerRec_t *self_, uint16_t capture);
void timerIn_dualCaptureEventStore( timerCCHandlerRec_t *self_, uint16_t capture);

struct timerQueueRec_s;

// check buffer length assumption
#if TIMERIN_QUEUE_LEN & (TIMERIN_QUEUE_LEN-1)
# error "TIMERIN_QUEUE_LEN must be power of 2"
#endif

void timerIn_Config(timerInputRec_t* self, const timerHardware_t* timHw, channelType_t owner, int priority, callbackRec_t *callback, struct timerQueueRec_s* timer, uint16_t flags)
{
    self->timHw=timHw;
    self->tim=timHw->tim;
    self->flags=flags;
    self->callback=callback;
    self->timer=timer;
    self->timeout=0;
    if(self->flags&TIMERIN_QUEUE_DUALTIMER) {
        // todo - mark second channel as used
        timerChInit(timHw, owner, priority);
        self->CCR=timerChCCRLo(timHw);
        timerChCCHandlerInit(&self->edgeLoCb, timerIn_dualCaptureEventStart);
        timerChCCHandlerInit(&self->edgeHiCb, timerIn_dualCaptureEventStore);
    } else {
        timerChInit(timHw, owner, priority);
        self->CCR=timerChCCR(timHw);
        timerChCCHandlerInit(&self->edgeLoCb, timerIn_timerCaptureEvent);        
    }
    timerIn_Restart(self);    
}

void timerIn_Release(timerInputRec_t* self)
{
    if(self->flags&TIMERIN_QUEUE_DUALTIMER)
        timerChConfigCallbacksDual(self->timHw, NULL, NULL, NULL);
    else 
        timerChConfigCallbacks(self->timHw, NULL, NULL);
}

void timerIn_Restart(timerInputRec_t* self)
{
    const timerHardware_t* timHw=self->timHw;
    if(self->flags&TIMERIN_QUEUE_DUALTIMER) {
        timerChConfigICDual(timHw, self->flags&TIMERIN_RISING, 0);
        timerChConfigGPIO(timHw, (self->flags&TIMERIN_IPD)?Mode_IPD:Mode_IPU);
        timerChConfigCallbacksDual(timHw, &self->edgeLoCb, &self->edgeHiCb, NULL);
    } else {
        if(self->flags&TIMERIN_RISING) 
            self->flags&=~TIMERIN_FLAG_HIGH;
        else 
            self->flags|=TIMERIN_FLAG_HIGH;
        timerChConfigIC(timHw, self->flags&TIMERIN_RISING, 0);
        timerChConfigGPIO(timHw, (self->flags&TIMERIN_IPD)?Mode_IPD:Mode_IPU); 
        timerChConfigCallbacks(timHw, &self->edgeLoCb, NULL);
    }
}

void timerIn_timerCaptureEvent(timerCCHandlerRec_t *self_, uint16_t capture) {
    timerInputRec_t* self=container_of(self_, timerInputRec_t, edgeLoCb);

    // check buffer space first
    unsigned nxt=(self->qhead+1)%TIMERIN_QUEUE_LEN;
    if(nxt==self->qtail) {
        // do not change polarity if queue is full. Next edge will be ignored
        // this way buffer stays synchronized
        return;
    }
    // toggle polarity if requested
    if(self->flags&TIMERIN_POLARITY_TOGGLE) {
        self->flags^=TIMERIN_FLAG_HIGH;
        timerChICPolarity(self->timHw, !(self->flags&TIMERIN_FLAG_HIGH));
    }
    // store received value into buffer
    self->queue[self->qhead]=capture;
    self->qhead=nxt;
    
    // start timer if requested
    if(self->flags&TIMERIN_TIMEOUT_FIRST) {
        self->flags&=~TIMERIN_TIMEOUT_FIRST;
        timerQueue_Start(self->timer, (capture-self->tim->CNT)+self->timeout);
    }
    if(!(self->flags&TIMERIN_QUEUE_BUFFER) 
       || (((self->qhead - self->qtail) & (TIMERIN_QUEUE_LEN-1)) > TIMERIN_QUEUE_HIGH) ) {   // flush only if queue is getting full
        callbackTrigger(self->callback);
    } 
}

void timerIn_dualCaptureEventStart(timerCCHandlerRec_t *self_, uint16_t capture) {
    timerInputRec_t* self=container_of(self_, timerInputRec_t, edgeLoCb);
    if(self->flags&TIMERIN_TIMEOUT_FIRST) {
        self->flags&=~TIMERIN_TIMEOUT_FIRST;
        timerQueue_Start(self->timer, (capture-self->tim->CNT)+self->timeout);
        timerChITConfigDualLo(self->timHw, DISABLE);
    }
}

void timerIn_dualCaptureEventStore(timerCCHandlerRec_t *self_, uint16_t capture) {
    timerInputRec_t* self=container_of(self_, timerInputRec_t, edgeHiCb);
    // two captured values are ready now
    unsigned nxt=(self->qhead+2)%TIMERIN_QUEUE_LEN; 
    if(nxt==(self->qtail&~1)) {  
        // both edges are discarded, so queue stays consistent
        // wake receiver, queue is full
        callbackTrigger(self->callback);
        return;
    }
    self->queue[self->qhead]=*self->CCR;
    self->queue[self->qhead+1]=capture;  // or self->CCR[3] if 16bit CCR / CCR[1] if 32bit ccr.
    self->qhead=nxt;

    if(!(self->flags&TIMERIN_QUEUE_BUFFER) 
       || (((self->qhead - self->qtail) & (TIMERIN_QUEUE_LEN-1)) > TIMERIN_QUEUE_HIGH) ) {   // flush only if queue is getting full
        callbackTrigger(self->callback);
    }
}

// prepare input timeout
// DUALTIMER - triggered by first edgeLo transition
// normal mode - triggered by first captured edge (setup trigger accordingly)
// return true if timeout is armed, false is something already happened and caller should check again

bool timerIn_ArmEdgeTimeout(timerInputRec_t* self) {
    bool ret;
    uint8_t saved_basepri = __get_BASEPRI();
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY)); asm volatile ("" ::: "memory"); 
    if(self->qhead==self->qtail) { // wait only in queue is empty
        self->flags|=TIMERIN_TIMEOUT_FIRST;
        if(self->flags&TIMERIN_QUEUE_DUALTIMER)
            timerChITConfigDualLo(self->timHw, ENABLE);   // TODO - check that IT flag is cleared correctly 
                                                          // (it is cleared by reading corresponding CCR on store)
        ret=true;
    } else {
        ret=false;
    }
    __set_BASEPRI(saved_basepri);
    return ret;
}

// only on/off implemented now
void timerIn_SetBuffering(timerInputRec_t *self, short buffer) {
    uint8_t saved_basepri = __get_BASEPRI();
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY)); asm volatile ("" ::: "memory"); 
    if(buffer) {
        self->flags|=TIMERIN_QUEUE_BUFFER;
    } else {
        self->flags&=~TIMERIN_QUEUE_BUFFER;
    }
    __set_BASEPRI(saved_basepri);
}


bool timerIn_QPeek(timerInputRec_t* self, uint16_t* capture, uint16_t* flags)
{
    if(self->qhead==self->qtail)
        return false;
    *capture=self->queue[self->qtail];
    *flags=(self->qtail&1)?0:TIMERIN_FLAG_HIGH;
    self->qtail=(self->qtail+1)%TIMERIN_QUEUE_LEN;
    return true;
}

void timerIn_QPop(timerInputRec_t* self)
{
    self->qtail=(self->qtail+1)%TIMERIN_QUEUE_LEN;
}

// queue position must be be always even when using these functions
bool timerIn_QPeek2(timerInputRec_t* self, uint16_t* capture1, uint16_t* capture2)
{
    if(self->qhead==(self->qtail&~1))
        return false;
    *capture1=self->queue[self->qtail];
    *capture2=self->queue[self->qtail+1];  // access is aligned, so this is OK
    return true;
}

void timerIn_QPop2(timerInputRec_t* self)
{
    self->qtail=(self->qtail+2)%TIMERIN_QUEUE_LEN;
}

int timerIn_QLen(timerInputRec_t* self) {
    return (self->qhead-self->qtail)&(TIMERIN_QUEUE_LEN-1);
}

uint16_t timerIn_getTimCNT(timerInputRec_t *self) {
    return self->tim->CNT;
}

