
// define pin definition structures here

#define IO_DEF_DEFINE 1

#include "io.h"
#include "io_def.h"

#undef IO_DEF_DEFINE
#define IO_DEF_DEFINE 2

#ifdef STM32F10X
# include "drivers/io_stm32f10x.h"
#endif

#ifdef STM32F303xC
# include "drivers/io_stm32f30x.h"
#endif
