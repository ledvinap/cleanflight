#pragma once

#include <stdbool.h>

#include "drivers/gpio.h"
#include "drivers/resource.h"

// IO pin identification
typedef struct ioRec_s ioRec_t;   // opaque type to represent IO pin
typedef uint8_t ioTag_t;          // packet tag to specify IO pin

// all available pins may be referenced using pinid (IO_PA1, IO_PB10, IO_NONE)
// preprocessor is used to convert it to requested C data
// compile-time error is generated if requexted pin is not available

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)
// expand pinid to ioRec_t* (possibly NULL)
#define IO_REC(pinid) DEFIO_REC(pinid)

// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations (and are CPU independent)

typedef uint8_t ioConfig_t;  // packed IO configuration
#if defined(STM32F10X)

// mode is using bits 6-2
# define IO_CONFIG(mode, speed) ((mode) | (speed))

# define IOCFG_OUT_PP             IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_2MHz)
# define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_Out_OD,      GPIO_Speed_2MHz)
# define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_2MHz)
# define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF_OD,       GPIO_Speed_2MHz)
# define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IPD,         GPIO_Speed_2MHz)
# define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IPU,         GPIO_Speed_2MHz)
# define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_2MHz)
#elif defined(STM32F303xC)

// define it here to be compatible with STM32F10X
# define GPIO_Speed_10MHz GPIO_Speed_Level_1  // Fast Speed:10MHz
# define GPIO_Speed_2MHz  GPIO_Speed_Level_2  // Medium Speed:2MHz (same as zero)
# define GPIO_Speed_50MHz GPIO_Speed_Level_3  // High Speed:50MHz

# define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

# define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
# define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
# define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
# define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
# define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
# define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
# define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             PIO_PuPd_NOPULL)

#endif

// declare available IO pins. Available pins are specified per target
#include "io_def.h"
#include "target_io.h"

bool IORead(ioRec_t *io);
void IOWrite(ioRec_t *io, bool value);
void IOHi(ioRec_t *io);
void IOLo(ioRec_t *io);
void IOToggle(ioRec_t *io);


void IOInit(ioRec_t *io, resourceOwner_t owner, resourceType_t resources);
void IORelease(ioRec_t *io);
resourceOwner_t IOGetOwner(ioRec_t *io);
resourceType_t IOGetResources(ioRec_t *io);
ioRec_t* IOGetByTag(ioTag_t tag);

void IOConfigGPIO(ioRec_t *io, ioConfig_t cfg);
#if defined(STM32F303xC)
void IOConfigGPIOAF(ioRec_t *io, ioConfig_t cfg, uint8_t af);
#endif

void IOInitGlobal(void);
