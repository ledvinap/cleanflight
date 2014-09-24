/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "gpio.h"

#include "callback.h"

#define TIMER_IRQ_PRIORITY 0
#define TIMER_IRQ_SUBPRIORITY 1

#ifdef CHEBUZZF3
#define USABLE_TIMER_CHANNEL_COUNT 18
#endif

#ifdef CC3D
#define USABLE_TIMER_CHANNEL_COUNT 12
#endif

#if !defined(USABLE_TIMER_CHANNEL_COUNT)
#define USABLE_TIMER_CHANNEL_COUNT 14
#endif

#if defined(STM32F303xC)
typedef uint32_t timCCR_t;
#elif defined(STM32F10X_MD)
typedef uint16_t timCCR_t;
#else 
# error "Unknown CPU defined"
#endif

#if defined(STM32F303xC)
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
#elif defined(STM32F10X)
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
#else 
# error "Unknown CPU defined"
#endif

typedef void timerCCCallbackPtr(void* data, uint16_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
} timerHardware_t;

extern const timerHardware_t timerHardware[];
extern const timerHardware_t timerQueueHardware;

#define TIMERQUEUE_FLAG_QUEUED 1

struct timerQueueRec_s;
typedef void timerQueueCallbackFn(struct timerQueueRec_s *self);

typedef struct timerQueueRec_s {
    uint16_t time;
    uint16_t flags;
    timerQueueCallbackFn *callbackFn;
} timerQueueRec_t;

#define TIMERIN_QUEUE_LEN (1<<5)
#define TIMERIN_QUEUE_HIGH (TIMERIN_QUEUE_LEN-5)  // high mark for que, flush if queue content is longer

#define TIMERIN_FLAG_HIGH         0x0001
#define TIMERIN_FLAG_TIMER        0x0002

#define TIMERIN_POLARITY_TOGGLE   0x0100    // Capture both edges (toggle polarity on each capture)
#define TIMERIN_QUEUE_BUFFER      0x0200    // only wake processissing when buffer is above high mark
#define TIMERIN_IPD               0x0400    // Configure input as pulldown

typedef struct timerData_Input {
    uint32_t queue[TIMERIN_QUEUE_LEN];
    uint8_t qhead;
    uint8_t qtail;
    uint32_t flags;
    const timerHardware_t* timHw;
    TIM_TypeDef *tim;
    callbackRec_t *callback;
} timerData_Input_t;

#define TIMEROUT_QUEUE_LEN (1<<5)
#define TIMEROUT_QUEUE_LOW 5

#define TIMEROUT_FLAG_ACTIVE   0x0001          // set active level for this interval
#define TIMEROUT_FLAG_WAKE     0x0002          // generate callback event when this interval starts

#define TIMEROUT_RUNNING       0x0100
#define TIMEROUT_INVERTED      0x0200          // output is inverted
#define TIMEROUT_WAKEONEMPTY   0x0400          // wake caller on last interval end
#define TIMEROUT_RESTART       0x0800          // TEST - interval was inserted in last period, do restart after timeout


typedef struct timerData_Output {
    struct {
        uint16_t delta;
        uint16_t tag;
    } queue[TIMEROUT_QUEUE_LEN];
    uint8_t qhead;                             // queue head used for sending data
    uint8_t qheadUnc;                          // index of uncommitted data head
    uint8_t qtail;
    uint16_t flags;
    const timerHardware_t* timHw;
    TIM_TypeDef *tim;
    volatile timCCR_t* timCCR;
    callbackRec_t *callback;
} timerData_Output_t;


void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, void* data, timerCCCallbackPtr *edgeCallback);
void configureTimerChannelCallbacks(TIM_TypeDef *tim, uint8_t channel, void* data, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback);

void timerChConfigIC(const timerHardware_t *channel, bool polarityRising);
void timerChICPolarity(const timerHardware_t *channel, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool outInvert);
void timerChCfgGPIO(const timerHardware_t* timHw, GPIO_Mode mode);
void timerChCfgCallbacks(const timerHardware_t *channel, void* data, timerCCCallbackPtr *edgeCallback, timerCCCallbackPtr *overflowCallback);
void timerConfigHandled(const timerHardware_t *timHw); // TODO?

void timerQueue_Init(void);
void timerQueue_Config(timerQueueRec_t *self, timerQueueCallbackFn *callbackFn);
void timerQueue_Start(timerQueueRec_t *self, int16_t timeout);

void timerOut_Config(timerData_Output_t *self, const timerHardware_t *timHw, callbackRec_t *callback, uint16_t tim_OCPolarity);
void timerOut_Release(timerData_Output_t *self);
void timerOut_Restart(timerData_Output_t *self);
short timerOut_QSpace(timerData_Output_t *self);
short timerOut_QLen(timerData_Output_t *self);
bool timerOut_QPush(timerData_Output_t *self, uint16_t delta, uint16_t flags);
void timerOut_QCommit(timerData_Output_t *self);


void timerIn_Config(timerData_Input_t *self, const timerHardware_t *timHw, callbackRec_t *callback, uint16_t flags);
void timerIn_Release(timerData_Input_t *self);
void timerIn_Restart(timerData_Input_t *self);
void timerIn_Polarity(timerData_Input_t *self , uint16_t tim_ICPolarity);
void timerIn_Flush(timerData_Input_t *self);
bool timerIn_QPop(timerData_Input_t *self, uint16_t *capture, uint16_t *flags);
void timerIn_SetBuffering(timerData_Input_t *self, short buffer);
uint16_t timerIn_getTimCNT(timerData_Input_t *self);
