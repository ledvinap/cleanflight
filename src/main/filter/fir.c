#include <stdint.h>

#include "common/utils.h"
#include "common/maths.h"

#include "fir.h"

int16_t filterApplyFIRi16(int16_t input, filterStateFIRi16_t *state, const filterConfigFIRi16_t *config)
{
    if(state->head > 0)
        state->head--;
    else
        state->head = FILTER_FIR_I16_LENGTH - 1;
    state->state[state->head] = input;
    const int16_t *fc = config->coef;
    int len = config->taps;
    int32_t accu = 0;
    // data till end of buffer
    int len1 = MIN(FILTER_FIR_I16_LENGTH - state->head, len);
    len -= len1;
    for(int16_t *ival = state->state + state->head; len1; ival++, fc++, len1--)
        accu += *ival * (*fc);
    // data from start of buffer
    for(int16_t *ival = state->state; len; ival++, fc++, len--)
        accu += *ival * (*fc);
    // scale accumulator back
    accu >>= FILTER_FIR_I16_COEF_SHIFT;
    // saturate to int16_t
    if(accu > 0x7fff) accu = 0x7fff;
    else if(accu < -0x8000) accu = -0x8000;
    return accu;
}

void filterResetFIRi16(int16_t initValue, filterStateFIRi16_t *state, const filterConfigFIRi16_t *config)
{
    UNUSED(config);
    state->head=0;
    for(int i=0; i < FILTER_FIR_I16_LENGTH; i++)
        state->state[i] = initValue;
}

