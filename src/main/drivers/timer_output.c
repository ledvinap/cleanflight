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
#include "timer_output.h"

// check buffer length assumption
#if TIMEROUT_QUEUE_LEN & (TIMEROUT_QUEUE_LEN-1)
# error "TIMEROUT_QUEUE_LEN must be power of 2"
#endif

void timerOut_timerCompareEvent(timerCCHandlerRec_t *self_, uint16_t compare);

void timerOut_Config(timerOutputRec_t* self, const timerHardware_t* timHw, channelType_t owner, int priority, callbackRec_t *callback, uint16_t flags)
{
    self->timHw=timHw;
    self->tim=timHw->tim;
    self->timCCR=timerChCCR(timHw);
    self->callback=callback;
    self->flags=flags;
    timerChInit(timHw, owner, priority);
    timerChCCHandlerInit(&self->compareCb, timerOut_timerCompareEvent);

    timerOut_Restart(self);
}

void timerOut_Release(timerOutputRec_t* self)
{
    self->flags&=~(TIMEROUT_RUNNING|TIMEROUT_RESTART);
    timerChConfigCallbacks(self->timHw, NULL, NULL);
    timerChConfigIC(self->timHw, false, 0);   // tristate channel - TODO - check pullup/pulldown
}

void timerOut_Restart(timerOutputRec_t* self)
{
    self->qhead=self->qheadUnc=self->qtail=0;
    self->qtailWake=~0;
    
    timerChConfigOC(self->timHw, true, self->flags&TIMEROUT_START_HI);
    timerChConfigGPIO(self->timHw, Mode_AF_PP);
    timerChConfigCallbacks(self->timHw, &self->compareCb, NULL);
}

void timerOut_timerCompareEvent(timerCCHandlerRec_t *self_, uint16_t compare)
{
    UNUSED(compare);
    timerOutputRec_t *self=container_of(self_, timerOutputRec_t, compareCb);

    if(self->flags&TIMEROUT_RUNNING) {
        if(self->flags&TIMEROUT_RESTART) {   // data was added too late, start new pulse train
            self->flags&=~TIMEROUT_RESTART;
            TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, TIM_OCMode_Toggle);
            *self->timCCR=self->tim->CNT+2;   // not sure what happens when CNT is written into CCR. This should work fine (we have at least 72 ticks to spare)
        } else if(self->qtail!=self->qhead) { // got something to send
            *self->timCCR+=self->queue[self->qtail];
            self->qtail=(self->qtail+1)%TIMEROUT_QUEUE_LEN;
            if(self->qtail==self->qhead) { // last interval, disable polarity change
                TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, TIM_OCMode_Timing);
            }
            if(self->qtailWake==self->qtail)  // user wants to be woken up
                callbackTrigger(self->callback);
        } else {
            digitalToggle(GPIOA, Pin_8);
            self->flags&=~TIMEROUT_RUNNING;
            if(self->flags&TIMEROUT_WAKEONEMPTY) 
                callbackTrigger(self->callback);
            // TODO - may disable channel irq here
        }
    }
}

short timerOut_QLen(timerOutputRec_t *self)
{
    return (self->qhead-self->qtail)&(TIMEROUT_QUEUE_LEN-1); 
}

short timerOut_QSpace(timerOutputRec_t *self)
{
    return TIMEROUT_QUEUE_LEN-1-timerOut_QLen(self);   // one entry is always empty
}

bool timerOut_QPush(timerOutputRec_t *self, uint16_t delta, uint16_t flags)
{
    UNUSED(flags);
    uint8_t nxt=(self->qheadUnc+1)%TIMEROUT_QUEUE_LEN;   
    if(nxt==self->qtail) {       // caller should check for this!
        return false;
    }
    self->queue[self->qheadUnc]=delta;
    self->qheadUnc=nxt;
    
    return true;
} 

void timerOut_QCommit(timerOutputRec_t *self)
{
    if(self->qheadUnc==self->qhead)   // no uncommited data
        return;
    // we need to to be atomic here
    register uint8_t saved_basepri= __get_BASEPRI();
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY));
    // and force compiler to discard any value read outside of atomic section
    asm volatile ("" ::: "memory"); 
    if(self->flags&TIMEROUT_RUNNING) {
        if(self->qhead==self->qtail) { // last interval in progress
            self->flags|=TIMEROUT_RESTART;
        }
        self->qhead=self->qheadUnc;  // commit prepared data
        // TODO - better wake handling
        // wake exactly on TIMEROUT_QUEUE_LOW items
    } else {
        *self->timCCR=self->tim->CNT-1;   // make sure there is no compare event soon
        // TODO - maybe clear timer ISR flag here if there was compare match
        self->flags|=TIMEROUT_RUNNING;
        // TODO - can't commit single interval this way (only delay, no toggle)
        TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, TIM_OCMode_Toggle);
        *self->timCCR=self->tim->CNT+2;     // not sure what happens when CNT is written into CCR. This should work fine (we have at lease 72 ticks to spare)
        // TODO - maybe we missed compare because higher priority IRQ was served here
        // we should retry unless CCR<CNT or irq flag is set
        // this will work is timer interrupt high enough
        // or we could disable all interrupts, ideally only for read/write instruction
        // [same assumption is in IRQ handler]
        self->qhead=self->qheadUnc;
    }
    if(self->flags&TIMEROUT_WAKEONLOW)
        self->qtailWake=(self->qhead-TIMEROUT_QUEUE_LOW)&(TIMEROUT_QUEUE_LEN-1);
    __set_BASEPRI(saved_basepri);
}
