
#pragma once

// simple pin-debug support
// define macros to use avalable pins to signal some event/state
// in normal build there is zero overhead (make sure this holds when making changes)
// select here which events(defines) are output to available pins
// there is no guarantee that this works, only that it will not break anything in production build
// usability is preffered to portability/corectness/etc

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"

#define DBG_PIN_1 true, GPIOB, Pin_6             // PWM11
#define DBG_PIN_2 true, GPIOB, Pin_9             // PWM14
#define DBG_PIN_3 true, GPIOA, Pin_0             // PWM1 (RC_CH1)
#define DBG_PIN_4 true, GPIOA, Pin_8             // PWM9 (PWM1)

#define DBG_NONE false, 0, 0

#define DBP_TIMER                            DBG_PIN_1
#define DBP_CALLBACK                         DBG_PIN_2

#define DBP_SOFTSERIAL_RXPROCESS             DBG_PIN_3     // in softSerialRxProcess
#define DBP_SOFTSERIAL_RXWAIT_SYMBOL         DBG_PIN_4     // waiting for whole symbol to arrive
#define DBP_TIMERINPUT_EDGEDELAY             DBG_PIN_4     // waiting for whole symbol to arrive

#define DBP_TELEMETRY_SPORT_REPLAYWAIT       DBG_NONE

static inline void pinDbgHi(bool enable, GPIO_TypeDef *gpio, uint16_t pin)
{
    if(enable) digitalHi(gpio, pin);
}

static inline void pinDbgLo(bool enable, GPIO_TypeDef *gpio, uint16_t pin)
{
    if(enable) digitalLo(gpio, pin);
}

static inline void pinDbgToggle(bool enable, GPIO_TypeDef *gpio, uint16_t pin)
{
    if(enable) digitalToggle(gpio, pin);
}

void pinDebugInit(void);

#define __UNIQL_CONCAT2(x,y) x ## y
#define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
#define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)

// assert pin for duration of enclosing block (local variable scope)
#define PIN_DBG_BLOCK(def)                                              \
    __extension__ void  __UNIQL(__pinDbgBlockEnd)(char *u __attribute__ ((unused))) {  pinDbgLo(def); } \
    char  __attribute__((__cleanup__(__UNIQL(__pinDbgBlockEnd))))       \
    __UNIQL(__pinDbgBlock);                                             \
    pinDbgHi(def);
