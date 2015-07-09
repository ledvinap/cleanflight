#include <stdint.h>
#include <string.h>

#include "common/maths.h"

#include "io/serial_msp.h"

#include "blackbox/blackbox_msp.h"

#define BBMSP_BLOCK 128

uint8_t bbmsp_buffer[BBMSP_BLOCK];
int bbmsp_length = 0;

void bbmspWriteByte(uint8_t byte)
{
    bbmsp_buffer[bbmsp_length++] = byte;
    if(bbmsp_length >= BBMSP_BLOCK)
        bbmspFlush();
}

int bbmspWrite(const uint8_t *data, unsigned len)
{
    int ret = len;
    while(len) {
        int wlen = MIN((int)len, BBMSP_BLOCK - bbmsp_length);
        memcpy(bbmsp_buffer + bbmsp_length, data, wlen);
        bbmsp_length += wlen; data += wlen; len -= wlen;
        if(bbmsp_length >= BBMSP_BLOCK)
            bbmspFlush();
    }
    return ret;
}

bool bbmspFlush(void)
{
    if(bbmsp_length) {
        sendMspBlackbox(bbmsp_buffer, bbmsp_length);
        bbmsp_length = 0;
    }
    return true;
}

void bbmspInfo(int cmd)
{
    sendMspBlackboxInfo(cmd);
}
