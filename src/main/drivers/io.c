
#include "common/utils.h"

#include "target_io.h"
#include "target.h"
#include "timer.h"


const timerHardware_t *getIOHw(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx < USABLE_TIMER_CHANNEL_COUNT + 2) return &timerHardware[idx];  // TODO!!!
    else return NULL;
}
