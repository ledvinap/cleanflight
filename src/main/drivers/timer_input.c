#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "callback.h"
#include "system.h"

#include "timer.h"

/* timer input fast handler. Input signal generates capture events,
   captured times are augmented with input state and stored in receive queue
*/
void timerIn_timerCaptureEvent(void *self_, uint16_t capture);

void timerIn_Config(timerData_Input_t* self, const timerHardware_t* timHw, callbackRec_t *callback, uint16_t flags)
{
    self->timHw=timHw;
    self->tim=timHw->tim;
    self->flags=flags;
    self->callback=callback;
    timerChConfigIC(timHw, !(self->flags&TIMERIN_FLAG_HIGH));
    timerChCfgGPIO(timHw, self->flags&TIMERIN_IPD?Mode_IPD:Mode_IPU); 
    timerChCfgCallbacks(timHw, (void*)self, timerIn_timerCaptureEvent, NULL);
    timerConfigHandled(timHw);
}

void timerIn_Release(timerData_Input_t* self)
{
    timerChCfgCallbacks(self->timHw, NULL, NULL, NULL);
}

void timerIn_Restart(timerData_Input_t* self)
{
    timerChConfigIC(self->timHw, !(self->flags&TIMERIN_FLAG_HIGH));
    timerChCfgGPIO(self->timHw, self->flags&TIMERIN_IPD?Mode_IPD:Mode_IPU); 
    timerChCfgCallbacks(self->timHw, (void*)self, timerIn_timerCaptureEvent, NULL);
}

void timerIn_timerCaptureEvent(void *self_, uint16_t capture) {
    timerData_Input_t* self=(timerData_Input_t*)self_;
    // keep old tolarity for tag
    uint16_t polarity=self->flags&TIMERIN_FLAG_HIGH;
    // toggle polarity if requested
    if(self->flags&TIMERIN_POLARITY_TOGGLE) {
        self->flags^=TIMERIN_FLAG_HIGH;
        timerChICPolarity(self->timHw, !(self->flags&TIMERIN_FLAG_HIGH));
    }
    // store received value into buffer
    uint8_t next=(self->qhead+1)%TIMERIN_QUEUE_LEN;
    if(next==self->qtail) {
        digitalToggle(GPIOB, Pin_8);
        return;
    }
    uint16_t tag=polarity;
    self->queue[self->qhead].tag=tag;
    self->queue[self->qhead].capture=capture;
    self->qhead=next;
    
    if(!(self->flags&TIMERIN_QUEUE_BUFFER) 
       || (((self->qhead - self->qtail) & (TIMERIN_QUEUE_LEN-1)) > TIMERIN_QUEUE_HIGH) ) {   // flush only if queue is getting full
        callbackTrigger(self->callback);
    } 
}

// capture current timer into queue and wake up handler
void timerIn_Flush(timerData_Input_t *self)
{
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY)); asm volatile ("" ::: "memory"); 
    uint16_t capture=self->tim->CNT;
    // check if overflow occured after reading CNT. Only store value if no capture occured. Trigger store in both cases
    if(!(self->tim->SR&(TIM_IT_CC1<<(self->timHw->channel>>2)))) {
        // store received value into buffer
        uint8_t next=(self->qhead+1)%TIMERIN_QUEUE_LEN;
        if(next!=self->qtail) {
            uint16_t tag=(self->flags&TIMERIN_FLAG_HIGH)|TIMERIN_FLAG_TIMER;
            self->queue[self->qhead].tag=tag;
            self->queue[self->qhead].capture=capture;
            self->qhead=next;
        }
    }
    // trigger callback. Either we stored a value or EdgeHandler will store it before priority is low enough 
    callbackTrigger(self->callback);
    __set_BASEPRI(0);
}

// only on/off implemented now
void timerIn_SetBuffering(timerData_Input_t *self, short buffer) {
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY)); asm volatile ("" ::: "memory"); 
    if(buffer) {
        self->flags|=TIMERIN_QUEUE_BUFFER;
    } else {
        self->flags&=~TIMERIN_QUEUE_BUFFER;
    }
    __set_BASEPRI(0);
}

bool timerIn_QPop(timerData_Input_t* self, uint16_t* capture, uint16_t* flags)
{
    if(self->qhead==self->qtail)
        return false;
    *capture=self->queue[self->qtail].capture;
    *flags=self->queue[self->qtail].tag;
    self->qtail=(self->qtail+1)%TIMERIN_QUEUE_LEN;
    return true;
}

uint16_t timerIn_getTimCNT(timerData_Input_t *self) {
    return self->tim->CNT;
}
