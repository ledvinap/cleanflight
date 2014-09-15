#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "callback.h"

#include "timer.h"

/* 
   Timeout generator for timerIn. Could be extended to support other timming tasks.
   Implemented as binary heap
 */
#define TIMERQUEUE_QUEUE_LEN 10
struct {
    timerQueueRec_t* heap[TIMERQUEUE_QUEUE_LEN];
    unsigned heapLen;
    volatile timCCR_t *timCCR;
    volatile uint16_t *timCNT;
} timerQueue;

callbackRec_t timerQueueCallback;

void timerQueue_TimerCompareEvent(void* self_, uint16_t compare);
void timerQueue_CallbackEvent(callbackRec_t *cb);
void timerQueue_Run(void);
static int timerQueue_QueueInsert(timerQueueRec_t *rec);
static void timerQueue_QueueDelete(timerQueueRec_t *rec);
static void timerQueue_QueueDeleteIdx(unsigned parent);

void timerQueue_Init(void)
{
    timerQueue.heapLen=0;
    timerQueue.timCCR=timerChCCR(&timerQueueHardware);
    timerQueue.timCNT=&timerQueueHardware.tim->CNT;
    timerConfigHandled(&timerQueueHardware);
    timerChConfigOC(&timerQueueHardware, false, false);
    timerChCfgCallbacks(&timerQueueHardware, NULL, timerQueue_TimerCompareEvent, NULL);
    callbackRegister(&timerQueueCallback, timerQueue_CallbackEvent);
}

// compare two timestamps circular timestamps. 
// result is negative when a<b, positive when a>b and zero if times are equal .. 

static inline int16_t tq_cmp_val(uint16_t a, uint16_t b) 
{
    return a-b;
}

static inline int16_t tq_cmp(const timerQueueRec_t *a, timerQueueRec_t *b)
{
    return tq_cmp_val(a->time, b->time);
}

void timerQueue_Config(timerQueueRec_t *self, timerQueueCallbackFn *callbackFn) 
{
    self->callbackFn=callbackFn;
    self->flags=0;
}

// start timer, trigger after timeout usec
void timerQueue_Start(timerQueueRec_t *self, int16_t timeout)
{
    if(self->flags&TIMERQUEUE_FLAG_QUEUED) {
        // TODO - this could be optimized - just update position
        timerQueue_QueueDelete(self);
    }
    self->time=*timerQueue.timCNT+timeout;
    if(!timerQueue_QueueInsert(self)) {
        // timer was inserted into head position - trigger callback to update CCR
        //  (running directly could he hard on stack)
        callbackTrigger(&timerQueueCallback);  
    }
}


// insert new timer into queue
// return position where new record was inserted
static int timerQueue_QueueInsert(timerQueueRec_t *rec)
{
    unsigned parent, child;
    child = timerQueue.heapLen++;
    while(child) {
        parent=(child - 1)/2;
        if(tq_cmp(timerQueue.heap[parent], rec)<=0) break;
        timerQueue.heap[child] = timerQueue.heap[parent];
        child=parent;
    }
    timerQueue.heap[child] = rec;
    rec->flags|=TIMERQUEUE_FLAG_QUEUED;
    return child;
}

static void timerQueue_QueueDelete(timerQueueRec_t *rec)
{
    for(unsigned i=0;i<timerQueue.heapLen;i++)
        if(timerQueue.heap[i]==rec) {
            timerQueue_QueueDeleteIdx(i);
            return;
        }
}
// remove element at given index from queue
static void timerQueue_QueueDeleteIdx(unsigned parent)
{
    if(timerQueue.heapLen==0) return; 
    unsigned child;
    timerQueueRec_t *last=timerQueue.heap[--timerQueue.heapLen];
    timerQueue.heap[parent]->flags&=~TIMERQUEUE_FLAG_QUEUED;
    while ((child = (2*parent)+1) < timerQueue.heapLen) {
        if (child + 1 < timerQueue.heapLen
            && tq_cmp(timerQueue.heap[child], timerQueue.heap[child+1])>=0)
            ++child;
        if(tq_cmp(last, timerQueue.heap[child])<=0) 
            break;
        timerQueue.heap[parent] = timerQueue.heap[child];
        parent = child;
    }
    timerQueue.heap[parent] = last;
}

void timerQueue_TimerCompareEvent(void* self_, uint16_t compare)
{
    UNUSED(self_);
    UNUSED(compare);
    // Only trigger callback here. Queue operations may be slow
    callbackTrigger(&timerQueueCallback); 
}

// callback function for timer queue
// - enqueue new handlers
// - trigger expired handlers 
// - prepare new timer compare event
// this function must be run (at least) on callbbak (pendSv) interrupt priority
void timerQueue_CallbackEvent(callbackRec_t *cb)
{
    UNUSED(cb);
    // run all tending timers
    timerQueue_Run();
}

void timerQueue_Run(void) 
{
check_again:
    // trigger all due timers, removing them from queue
    while(timerQueue.heapLen && tq_cmp_val(timerQueue.heap[0]->time, *timerQueue.timCNT)<0) {
        timerQueueRec_t *rec=timerQueue.heap[0];
        timerQueue_QueueDeleteIdx(0);                             // remove timer from queue first, so it is easy to reinsert it in callback function
        rec->callbackFn(rec);                                      // call regitered function
    }
    // replan timer    
    if(timerQueue.heapLen) {
        *timerQueue.timCCR=timerQueue.heap[0]->time;
        if(tq_cmp_val(timerQueue.heap[0]->time, *timerQueue.timCNT)<0) { 
            // maybe timer triggered, maybe not
            // simply check timer again to be sure
            goto check_again;
        }
    } else {
        // TODO - disable timer channel interrupt 
        // but triggering in next timer period does not hurt much ... 
    }
}
