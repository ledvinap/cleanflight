
#include "platform.h"
#include "common/utils.h"
#include "system.h"

#include "pin_debug.h"

#ifdef PINDEBUG

// use stdperiph interface, debug should be independent on rest of code

#define _ENTRY2(a,b,c) {b,c}
#define _ENTRY(...) _ENTRY2(__VA_ARGS__)
struct {
    GPIO_TypeDef *gpio;
    uint16_t pin;
} const pinDebugPins[] = {
#ifdef DBG_PIN_1
    _ENTRY(DBG_PIN_1),
#endif
#ifdef DBG_PIN_2
    _ENTRY(DBG_PIN_2),
#endif
#ifdef DBG_PIN_3
    _ENTRY(DBG_PIN_3),
#endif
#ifdef DBG_PIN_4
    _ENTRY(DBG_PIN_4),
#endif
};
#undef _ENTRY
#undef _ENTRY2

void pinDebugInit(void)
{
    for(unsigned i = 0; i < ARRAYLEN(pinDebugPins); i++) {
        // TODO - enable RCC
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = pinDebugPins[i].pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(pinDebugPins[i].gpio, &GPIO_InitStructure);

        // generate short pulse on debug pins.
        // debug pin order and correct connection will be easily visible on logic analyser
        GPIO_SetBits(pinDebugPins[i].gpio, pinDebugPins[i].pin);
        for(int i = 0; i < 100000; i++) asm volatile("\tnop\n");  // delay() may not be initialized yet
        GPIO_ResetBits(pinDebugPins[i].gpio, pinDebugPins[i].pin);
    }
}

bool pinDebugIsPinUsed(GPIO_TypeDef* gpio, uint16_t pin)
{
    for(unsigned i = 0;i < ARRAYLEN(pinDebugPins); i++)
        if((pinDebugPins[i].gpio == gpio) && (pinDebugPins[i].pin & pin))
            return true;
    return false;
}

#endif
