
#include "config/config_def.h"
#include "config/config_parser.h"

#define TYPE_PID (                               \
        pidDef,                                  \
        (uint16_t P),                            \
        (uint16_t I),                            \
        (uint16_t D)                             \
        )                                        \
    /**/


#define CONFIG_ESC_SERVO (                                              \
        escAndServoConfig,                                              \
        (struct pidDef_t pid, DFLT(({100, 50, 10}))),                   \
        (uint16_t minthrottle, COND(ALIENWII32, DFLT(1000)), DFLT(1150), MIN(0), \
         DOC("Set the minimum throttle command sent to the ESC. This is the minimum value that allows motors to run at a idle speed.") ), \
        (uint16_t maxthrottle, COND(ALIENWII32, DFLT(200)), DFLT(1850), \
         DOC("Maximum value for the ESCs at full power") ),             \
        (uint16_t mincommand, DFLT(1000),                               \
         DOC("Value for the ESCs when they are not armed") )            \
        )                                                               \
    /**/


CDEF_STRUCT_DECLARE(TYPE_PID);
CDEF_STRUCT_DECLARE(CONFIG_ESC_SERVO);
CDEF_METAINFO(TYPE_PID);
CDEF_METAINFO(CONFIG_ESC_SERVO);

CDEF_STRUCT(CONFIG_ESC_SERVO) test = {1,2,3};

int main(void)
{
    cdef_dump(CDEF_METAINFO_REF(CONFIG_ESC_SERVO), &test, 0);
    return 0;
}
