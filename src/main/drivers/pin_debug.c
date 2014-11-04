
#include "platform.h"
#include "common/utils.h"
#include "system.h"

#include "pin_debug.h"

#ifdef PINDEBUG

struct {
    bool enable;
    GPIO_TypeDef *gpio;
    uint16_t pin;
} pinDebugPins[] = {
    {DBG_PIN_1},
    {DBG_PIN_2},
    {DBG_PIN_3},
    {DBG_PIN_4},
};

void pinDebugInit(void)
{
    for(unsigned i=0; i < ARRAYLEN(pinDebugPins); i++) {
        gpio_config_t cfg;
        cfg.pin = pinDebugPins[i].pin;
        cfg.mode = Mode_Out_PP;
        cfg.speed = Speed_2MHz;
        gpioInit(pinDebugPins[i].gpio, &cfg);
        digitalHi(pinDebugPins[i].gpio, pinDebugPins[i].pin);
        delay(1);
        digitalLo(pinDebugPins[i].gpio, pinDebugPins[i].pin);
    }
}

bool pinDebugIsPinUsed(GPIO_TypeDef* gpio, uint16_t pin)
{
    for(int i=0;i<ARRAYLEN(pinDebugPins);i++)
        if((pinDebugPins[i].gpio == gpio) && (pinDebugPins[i].pin & pin))
            return true;
    return false;
}

#endif
