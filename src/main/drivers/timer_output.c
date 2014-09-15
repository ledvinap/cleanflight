#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "callback.h"
#include "system.h"

#include "timer.h"


void timerOut_timerCompareEvent(void *self_, uint16_t compare);

void timerOut_Config(timerData_Output_t* self, const timerHardware_t* timHw, callbackRec_t *callback, uint16_t flags)
{
    self->timHw=timHw;
    self->tim=timHw->tim;
    self->timCCR=timerChCCR(timHw);
    self->callback=callback;
    self->flags=flags;
    timerChConfigOC(timHw, true, self->flags&TIMEROUT_INVERTED);
    timerChCfgGPIO(timHw, Mode_AF_PP);
    timerChCfgCallbacks(timHw, (void*)self, timerOut_timerCompareEvent, NULL);
    timerConfigHandled(timHw);
}

void timerOut_Release(timerData_Output_t* self)
{
    self->flags&=~TIMEROUT_RUNNING;
    timerChCfgCallbacks(self->timHw, NULL, NULL, NULL);
    timerChConfigIC(self->timHw, 0);   // tristate channel
}

void timerOut_Restart(timerData_Output_t* self)
{
    self->qhead=self->qheadUnc=self->qtail=0;
    timerChConfigOC(self->timHw, true, self->flags&TIMEROUT_INVERTED);
    timerChCfgGPIO(self->timHw, Mode_AF_PP);
    timerChCfgCallbacks(self->timHw, (void*)self, timerOut_timerCompareEvent, NULL);
}

void timerOut_timerCompareEvent(void *self_, uint16_t compare)
{
    UNUSED(compare);
    timerData_Output_t *self=(timerData_Output_t*)self_;

    if(self->flags&TIMEROUT_RUNNING) {
        if(self->flags&TIMEROUT_RESTART) {   // data was added too late, resync output
            self->flags&=~TIMEROUT_RESTART;
            TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, (self->queue[self->qtail].tag&TIMEROUT_FLAG_ACTIVE)?TIM_OCMode_Active:TIM_OCMode_Inactive);
            *self->timCCR=self->tim->CNT+100;   // not sure what happens when CNT is written into CCR. This should work fine (we have 72 ticks to spare)
        } else if(self->qtail!=self->qhead) { // got something to send
            *self->timCCR+=self->queue[self->qtail].delta;
            bool trig=self->queue[self->qtail].tag&TIMEROUT_FLAG_WAKE;
            self->qtail=(self->qtail+1)%TIMEROUT_QUEUE_LEN;
            if(self->qtail!=self->qhead) { // we know polarity for next interval
                TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, (self->queue[self->qtail].tag&TIMEROUT_FLAG_ACTIVE)?TIM_OCMode_Active:TIM_OCMode_Inactive);
            }
            if(trig)  // set callbarch trigger here to save few ticks
                callbackTrigger(self->callback);
        } else {
            self->flags&=~TIMEROUT_RUNNING;
            if(self->flags&TIMEROUT_WAKEONEMPTY) 
                callbackTrigger(self->callback);
            // TODO - may disable channel irq here
        }
    }
}

short timerOut_QLen(timerData_Output_t *self)
{
    return (self->qhead-self->qtail)&(TIMEROUT_QUEUE_LEN-1); 
}

short timerOut_QSpace(timerData_Output_t *self)
{
    return TIMEROUT_QUEUE_LEN-1-timerOut_QLen(self);   // one entry is always empty
}

bool timerOut_QPush(timerData_Output_t *self, uint16_t delta, uint16_t flags)
{
    uint8_t nxt=(self->qheadUnc+1)%TIMEROUT_QUEUE_LEN;   
    if(nxt==self->qtail) {       // caller should check for this!
        return false;
    }
    self->queue[self->qheadUnc].delta=delta;
    self->queue[self->qheadUnc].tag=flags;
    self->qheadUnc=nxt;
    return true;
} 

void timerOut_QCommit(timerData_Output_t *self)
{
    if(self->qheadUnc==self->qhead)   // no uncommited data
        return;
    // we need to to be atomic here
    uint8_t saved_basepri= __get_BASEPRI();
    __set_BASEPRI(NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY));
    // and force compiler to discard qhead read outside of atomic section
    asm volatile ("" ::: "memory"); 
    if(self->flags&TIMEROUT_RUNNING) {
        if(self->qhead==self->qtail) { // last interval in progress
#if 0
            // qhead points to first uncommitted item, use it
            TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, (self->queue[self->qhead].tag&TIMEROUT_FLAG_ACTIVE)?TIM_OCMode_Active:TIM_OCMode_Inactive);
            // TODO - maybe there was compare match. It would be probably best to force output in that case
#else
            self->flags|=TIMEROUT_RESTART;
#endif
        }
        self->qhead=self->qheadUnc;  // commit prepared data
    } else {
        *self->timCCR=self->tim->CNT-1;   // make sure there is no compare event soon
        // TODO - maybe clear timer ISR flag here if there was compare match
        self->flags|=TIMEROUT_RUNNING;
        TIM_SelectOCxM_NoDisable(self->tim, self->timHw->channel, (self->queue[self->qtail].tag&TIMEROUT_FLAG_ACTIVE)?TIM_OCMode_Active:TIM_OCMode_Inactive);
        *self->timCCR=self->tim->CNT+5;     // not sure what happens when CNT is written into CCR. This should work fine (we have 72 ticks to spare)
        // TODO - maybe we missed compare because higher priority IRQ was served here
        // we should retry unless CCR<CNT or irq flag is set
        // or simply disable all interrupts, ideally only for read/write instruction
        self->qhead=self->qheadUnc;
    }
    __set_BASEPRI(saved_basepri);
}
