#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"


#include "drivers/nvic.h"

#include "exti.h"

#ifdef USE_EXTI

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelConfig_t;

extiChannelConfig_t extiChannelConfig[16]; // TODO exti channels

#ifdef STM32F10X
static uint8_t extiIRQn[] = {
    EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn,
    EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
    EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};
#endif

void EXTIInit(void)
{
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    memset(extiChannelConfig, 0, sizeof(extiChannelConfig));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->cb = fn;
}

void EXTIConfig(IOId_t pin, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger)
{
    int chIdx;
    chIdx = IO_GPIOPinIdx(pin);
    if(chIdx < 0)
        return;
    extiChannelConfig[chIdx].handler = cb;

    gpioExtiLineConfig(IO_GPIOPortSource(pin), IO_GPIOPinSource(pin));

    uint32_t extiLine = IO_EXTILine(pin);

    EXTI_ClearITPendingBit(extiLine);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiLine;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = trigger;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = extiIRQn[chIdx];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTIEnable(IOId_t pin, bool enable)
{
#if defined(STM32F10X)
    uint32_t extiLine = IO_EXTILine(pin);
    if(!extiLine)
        return;
    if(enable)
        EXTI->IMR |= extiLine;
    else
        EXTI->IMR &= ~extiLine;
#elif defined(STM32F30X)
    int extiLine = IO_EXTILine(pin);
    if(extiLine < 0)
        return;
    // assume extiLine < 32 (walid for all EXTI pins)
    if(enable)
        EXTI->IMR |= 1 << extiLine;
    else
        EXTI->IMR &= ~(1 << extiLine);
#else
# error "Unsupported target"
#endif
}

void EXTI_IRQHandler(void)
{
    uint32_t exti_active = EXTI->IMR & EXTI->PR;

    while(exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t mask = 1 << idx;
        extiChannelConfig[idx].handler->cb(extiChannelConfig[idx].handler);
        EXTI->PR = mask;  // clear pending mask (by writing 1)
        exti_active &= ~mask;
    }
}

#define _EXTI_IRQ_HANDLER(name)                 \
    void name(void) {                           \
        EXTI_IRQHandler();                      \
    }                                           \
    struct dummy                                \
    /**/

_EXTI_IRQ_HANDLER(EXTI1_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler);

#endif
