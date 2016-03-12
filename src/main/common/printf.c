/*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include <platform.h>

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/serial.h"
#include "io/serial.h"

#include "build_config.h"
#include "printf.h"

#ifdef REQUIRE_PRINTF_LONG_SUPPORT
#include "typeconversion.h"
#endif

static serialPort_t *printfSerialPort;

#ifdef REQUIRE_CC_ARM_PRINTF_SUPPORT

typedef int (*writef) (void *, const char *, int);
typedef void (*putcf) (void *, char);
static putcf stdout_putf;
static void *stdout_putp;

// print bf, padded from left to at least n characters. 
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, writef writef, int n, char z, char *bf)
{
    int written = 0;
#define CHUNK 8
    static const char fill_0[CHUNK] = "00000000";
    static const char fill_space[CHUNK] = "        ";
    const char *fc = z ?  fill_0 : fill_space;

    n -= strnlen(bf, n);
    while (n > 0) {
        int chunk = MIN(n, CHUNK);
        written += writef(putp, fc, chunk);
        n -= chunk;
    }
    written += writef(putp, bf, strlen(bf));
    return written;
#undef CHUNK
}

// retrun number of bytes written
int tfp_format(void *putp, writef writef, const char *fmt, va_list va)
{
    char bf[12];
    int written = 0;
    const char* processed = fmt;   // start of format string not yet written to output
    char ch;

    while ((ch = *fmt++)) {
        if (ch == '%') {
            // commit all unprocessed format string
            if(fmt - 1 != processed)
                written += writef(putp, processed, fmt - 1 - processed);

            char lz = 0;
#ifdef 	REQUIRE_PRINTF_LONG_SUPPORT
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#ifdef 	REQUIRE_PRINTF_LONG_SUPPORT
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
#endif
            switch (ch) {
            case 0:
                goto abort;
            case 'u':{
#ifdef 	REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 10, 0, bf);
                written += putchw(putp, writef, w, lz, bf);
                break;
            }
            case 'i':
            case 'd':{
#ifdef 	REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    li2a(va_arg(va, unsigned long int), bf);
                else
#endif
                    i2a(va_arg(va, int), bf);
                written += putchw(putp, writef, w, lz, bf);
                break;
            }
            case 'x':
            case 'X':
#ifdef 	REQUIRE_PRINTF_LONG_SUPPORT
                if (lng)
                    uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                else
#endif
                    ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                written += putchw(putp, writef, w, lz, bf);
                break;
            case 'c': {
                char val = va_arg(va, int);
                written += writef(putp, &val, 1);
                break;
            }
            case 's':
                written += putchw(putp, writef, w, 0, va_arg(va, char *));
                break;
            case '%':
                written += writef(putp, "%", 1);
                break;
            case 'n':
                *va_arg(va, int*) = written;
                break;
            default:
                break;
            }
            processed = fmt;
        }
    }
    // commit rest of format string
    if(fmt - 1 != processed)
        written += writef(putp, processed, fmt - 1 - processed);
  abort:;
    return written;
}

void init_printf(void *putp, void (*putf) (void *, char))
{
    stdout_putf = putf;
    stdout_putp = putp;
}

static int stdout_writef(void *putp, const char* data, int len)
{
    int n = len;
    while(n--)
        stdout_putf(putp, *data++);
    return len;
}

int tfp_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int written = tfp_format(stdout_putp, stdout_writef, fmt, va);
    va_end(va);
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    return written;
}


static int string_writef(void *p, const char* data, int len)
{
    memcpy(*((char **) p), data, len);
    *((char **) p) += len;
    return len;
}

int tfp_sprintf(char *s, const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    int written = tfp_format(&s, string_writef, fmt, va);
    string_writef(&s, "", 1);
    va_end(va);
    return written;
}


static void _putc(void *p, char c)
{
    UNUSED(p);
    serialWrite(printfSerialPort, c);
}

void printfSupportInit(void)
{
    init_printf(NULL, _putc);
}

#else

// keil/armcc version
int fputc(int c, FILE *f)
{
    // let DMA catch up a bit when using set or dump, we're too fast.
    while (!isSerialTransmitBufferEmpty(printfSerialPort));
    serialWrite(printfSerialPort, c);
    return c;
}

void printfSupportInit(void)
{
    // Nothing to do
}
#endif

void setPrintfSerialPort(serialPort_t *serialPort)
{
    printfSerialPort = serialPort;
}
