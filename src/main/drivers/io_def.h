#pragma once

#include "common/utils.h"

// return ioTag_t for given pinid
#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0xff

// return ioRec_t or NULL for given pinid
// TODO - some magic involved, maybe use TAG everywhere
// io_impl.h must be included is this macro is used
#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL

#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)
#define DEFIO_IO_ISEMPTY(io) (io == NULL)
// TODO - macro to check for pinid NONE

// get ioRec by index
#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))

// ioTag_t accessor macros
#define DEFIO_TAG_MAKE(gpioid, pin) (((gpioid) << 4) | (pin))
#define DEFIO_TAG_ISEMPTY(tag) ((tag) == 0xff)
#define DEFIO_TAG_GPIOID(tag) ((tag) >> 4)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)

// TARGET must define used pins
#include "target_io.h"
// include template-generated macros for IO pins
#include "io_def_generated.h"

