
#include "common/utils.h"

#include "target_io.h"
#include "target.h"
#include "timer.h"


const timerHardware_t *getIOHw(IOId_t id)
{
    unsigned idx = id - IO1;
    if(idx < USABLE_IO_CHANNEL_COUNT) return &timerHardware[idx];  // TODO!!!
    else return NULL;
}
