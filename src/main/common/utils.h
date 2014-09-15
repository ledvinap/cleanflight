
#pragma once

#include <stddef.h>

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

/*
http://resnet.uoregon.edu/~gurney_j/jmpc/bitwise.html
*/
#define BITCOUNT(x) (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)
#define BX_(x) ((x) - (((x)>>1)&0x77777777) - (((x)>>2)&0x33333333) - (((x)>>3)&0x11111111))

#define UNUSED(x) (void)(x)

#if 1
// ISO C version, byt no type checking
#define container_of(ptr, type, member) \
                      ((type *) ((char *)(ptr) - offsetof(type, member)))
#else
// this variant (from linux kernel) checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})
#endif
