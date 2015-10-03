#pragma once

#include "common/utils.h"

#define BOOST_PP_VARIADICS 1
#include <boost/preprocessor/comparison/less.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/debug/assert.hpp>
#include <boost/preprocessor/logical/bool.hpp>
#include <boost/preprocessor/punctuation/is_begin_parens.hpp>
#include <boost/preprocessor/seq/filter.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>

// there is DEFMAP__xy (x,y) macro defined for each PORT/PIN combination
// it may be used to attach aditional IO info (using tuple (a,y,io_data,...) )

// return ioTag_t
#define DEFIO_TAG(pinid) DEFIO_MAP_PINID(pinid, DEFIO_TAG_I, 0xff)
#define DEFIO_TAG_I(port, pin) DEFIO_IO_TAG(DEFIO_PORT_GPIOID(port), pin)


// return ioRec_t or NULL
#define DEFIO_REC(pinid) DEFIO_MAP_PINID(pinid, DEFIO_REC_I, NULL)
#define DEFIO_REC_I(port, pin) (&DEFIO_IO_REC_NAME(port, pin))


// expand to error if pinid is invalid or not supported by target
// expand to on_none if pinid is NONE
// otherwise expand to on_valid(port, pin)
#define DEFIO_MAP_PINID(pinid, on_valid, on_none)                       \
    BOOST_PP_IIF(DEFIO_PIN_ISNONE(pinid),                               \
                 DEFIO_MAP_PINID_NONE,                                  \
                 DEFIO_MAP_PINID_I)(pinid, on_valid, on_none)           \
    /**/

// called with NONE
#define DEFIO_MAP_PINID_NONE(pinid, on_valid, on_none) on_none

// check id pinid is defined
#define DEFIO_MAP_PINID_I(pinid, on_valid, on_none)                     \
    BOOST_PP_IIF(DEFIO_PIN_EXISTS(pinid),                               \
                 DEFIO_MAP_PINID_II,                                    \
                 DEFIO_MAP_PINID_UNKNOWN)(pinid, on_valid, on_none)     \
    /**/

// expand to something that will trigger compilation error (unknown identifier)
#define DEFIO_MAP_PINID_UNKNOWN(pinid, on_valid, on_none)       \
    BOOST_PP_CAT(error__pinid_is_invalid__, pinid)              \
    /**/

// pinid is defined, check if target supports it
#define DEFIO_MAP_PINID_II(pinid, on_valid, on_none)                    \
    BOOST_PP_IIF(DEFIO_PIN_SUPPORTED(pinid), \
                 DEFIO_MAP_PINID_III, \
                 DEFIO_MAP_PINID_UNSUPPORTED)(pinid, on_valid, on_none) \
    /**/

// expand to something that will trigger compilation error (unknown identifier)
#define DEFIO_MAP_PINID_UNSUPPORTED(pinid, on_valid, on_none)           \
    BOOST_PP_CAT(error__pinid_is_not_supported_on_target__, pinid)     \
    /**/

// checks passed, expand passed macro
#define DEFIO_MAP_PINID_III(pinid, on_valid, on_none)                   \
    on_valid(DEFIO_PINID_PORT(pinid), DEFIO_PINID_PIN(pinid))           \
    /**/


// ioTag_t accessor macros
#define DEFIO_IO_TAG(gpioid, pin) (((gpioid) << 4) | (pin))
#define DEFIO_IO_TAG_ISEMPTY(tag) ((tag) == 0xff)
#define DEFIO_IO_TAG_GPIOID(tag) ((tag) >> 4)
#define DEFIO_IO_TAG_PIN(tag) ((tag) & 0x0f)

// generate symbol (variable name) for fiven IO
#define DEFIO_IO_REC_NAME(port, pin) CONCAT(IO_P, CONCAT(port, pin))

// return port from pinid
#define DEFIO_PINID_PORT(pinid) BOOST_PP_TUPLE_ELEM(0, BOOST_PP_CAT(DEFMAP__, pinid))
// return pin from pinid
#define DEFIO_PINID_PIN(pinid) BOOST_PP_TUPLE_ELEM(1, BOOST_PP_CAT(DEFMAP__, pinid))
// return gpioid from port (DEFIO_PORT_GPIOID(A) == 0)
#define DEFIO_PORT_GPIOID(port) BOOST_PP_CAT(DEFIO_GPIOID__, port)


// check if pinid is NONE
#define DEFIO_PIN_ISNONE(pinid) BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(DEFIO_PIN_ISNONE__, pinid))
#define DEFIO_PIN_ISNONE__NONE ()

// check if pin is defined (DEFMAP__<pinid> is defined)
#define DEFIO_PIN_EXISTS(pinid) BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(DEFMAP__, pinid))


// check if pinid is supported on target (TARGET_IO_PORT<port> tuple contains pin number)
// pinid must exist
#define DEFIO_PIN_SUPPORTED(pinid)                                      \
    BOOST_PP_IIF(BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(TARGET_IO_PORT, DEFIO_PINID_PORT(pinid))), \
                 DEFIO_PIN_SUPPORTED_I,                                 \
                 DEFIO_PIN_SUPPORTED_0)(pinid)                          \
    /**/

#define DEFIO_PIN_SUPPORTED_I(pinid)                                    \
    PP_SEQ_CONTAINS(BOOST_PP_TUPLE_TO_SEQ(BOOST_PP_CAT(TARGET_IO_PORT, DEFIO_PINID_PORT(pinid))), DEFIO_PINID_PIN(pinid)) \
/**/
#define DEFIO_PIN_SUPPORTED_0(pinid) 0

// check if sequence contains element (BOOST_PP_EQUAL limitations apply)
#define PP_SEQ_CONTAINS(seq,elem) BOOST_PP_BOOL(BOOST_PP_SEQ_SIZE(BOOST_PP_SEQ_FILTER(PP_SEQ_CONTAINS_PRED, elem, seq)))
#define PP_SEQ_CONTAINS_PRED(s, data, elem) BOOST_PP_EQUAL(data,elem)

// map port letter to numeric ID
#define DEFIO_GPIOID__A 0
#define DEFIO_GPIOID__B 1
#define DEFIO_GPIOID__C 2
#define DEFIO_GPIOID__D 3
#define DEFIO_GPIOID__E 4
#define DEFIO_GPIOID__F 5

// map all available <port><pin> combinations to corresponding tuple
// may contain aditianal info in future (and thus become CPU or target dependent)
#define DEFMAP__PA0  (A,0)
#define DEFMAP__PA1  (A,1)
#define DEFMAP__PA2  (A,2)
#define DEFMAP__PA3  (A,3)
#define DEFMAP__PA4  (A,4)
#define DEFMAP__PA5  (A,5)
#define DEFMAP__PA6  (A,6)
#define DEFMAP__PA7  (A,7)
#define DEFMAP__PA8  (A,8)
#define DEFMAP__PA9  (A,9)
#define DEFMAP__PA10 (A,10)
#define DEFMAP__PA11 (A,11)
#define DEFMAP__PA12 (A,12)
#define DEFMAP__PA13 (A,13)
#define DEFMAP__PA14 (A,14)
#define DEFMAP__PA15 (A,15)

#define DEFMAP__PB0  (B,0)
#define DEFMAP__PB1  (B,1)
#define DEFMAP__PB2  (B,2)
#define DEFMAP__PB3  (B,3)
#define DEFMAP__PB4  (B,4)
#define DEFMAP__PB5  (B,5)
#define DEFMAP__PB6  (B,6)
#define DEFMAP__PB7  (B,7)
#define DEFMAP__PB8  (B,8)
#define DEFMAP__PB9  (B,9)
#define DEFMAP__PB10 (B,10)
#define DEFMAP__PB11 (B,11)
#define DEFMAP__PB12 (B,12)
#define DEFMAP__PB13 (B,13)
#define DEFMAP__PB14 (B,14)
#define DEFMAP__PB15 (B,15)

#define DEFMAP__PC0  (C,0)
#define DEFMAP__PC1  (C,1)
#define DEFMAP__PC2  (C,2)
#define DEFMAP__PC3  (C,3)
#define DEFMAP__PC4  (C,4)
#define DEFMAP__PC5  (C,5)
#define DEFMAP__PC6  (C,6)
#define DEFMAP__PC7  (C,7)
#define DEFMAP__PC8  (C,8)
#define DEFMAP__PC9  (C,9)
#define DEFMAP__PC10 (C,10)
#define DEFMAP__PC11 (C,11)
#define DEFMAP__PC12 (C,12)
#define DEFMAP__PC13 (C,13)
#define DEFMAP__PC14 (C,14)
#define DEFMAP__PC15 (C,15)

#define DEFMAP__PD0  (D,0)
#define DEFMAP__PD1  (D,1)
#define DEFMAP__PD2  (D,2)
#define DEFMAP__PD3  (D,3)
#define DEFMAP__PD4  (D,4)
#define DEFMAP__PD5  (D,5)
#define DEFMAP__PD6  (D,6)
#define DEFMAP__PD7  (D,7)
#define DEFMAP__PD8  (D,8)
#define DEFMAP__PD9  (D,9)
#define DEFMAP__PD10 (D,10)
#define DEFMAP__PD11 (D,11)
#define DEFMAP__PD12 (D,12)
#define DEFMAP__PD13 (D,13)
#define DEFMAP__PD14 (D,14)
#define DEFMAP__PD15 (D,15)

#define DEFMAP__PE0  (E,0)
#define DEFMAP__PE1  (E,1)
#define DEFMAP__PE2  (E,2)
#define DEFMAP__PE3  (E,3)
#define DEFMAP__PE4  (E,4)
#define DEFMAP__PE5  (E,5)
#define DEFMAP__PE6  (E,6)
#define DEFMAP__PE7  (E,7)
#define DEFMAP__PE8  (E,8)
#define DEFMAP__PE9  (E,9)
#define DEFMAP__PE10 (E,10)
#define DEFMAP__PE11 (E,11)
#define DEFMAP__PE12 (E,12)
#define DEFMAP__PE13 (E,13)
#define DEFMAP__PE14 (E,14)
#define DEFMAP__PE15 (E,15)

#define DEFMAP__PF0  (F,0)
#define DEFMAP__PF1  (F,1)
#define DEFMAP__PF2  (F,2)
#define DEFMAP__PF3  (F,3)
#define DEFMAP__PF4  (F,4)
#define DEFMAP__PF5  (F,5)
#define DEFMAP__PF6  (F,6)
#define DEFMAP__PF7  (F,7)
#define DEFMAP__PF8  (F,8)
#define DEFMAP__PF9  (F,9)
#define DEFMAP__PF10 (F,10)
#define DEFMAP__PF11 (F,11)
#define DEFMAP__PF12 (F,12)
#define DEFMAP__PF13 (F,13)
#define DEFMAP__PF14 (F,14)
#define DEFMAP__PF15 (F,15)

// define all available port structures
#define DEFIO_IO_DEFINE()                                               \
    DEFIO_APPLY_PORTS(BOOST_PP_TUPLE_EAT(), DEFIO_IO_DEFINE_I, BOOST_PP_TUPLE_EAT()) \
    struct dummy                                                        \
    /**/

// emit actual IO definitions and structures
#define DEFIO_IO_DEFINE_I(port, pin)                                    \
    ioRec_t DEFIO_IO_REC_NAME(port, pin)                                \
        __attribute__ ((section (".tab.io.rec." STR(port) BOOST_PP_IF(BOOST_PP_LESS(pin, 10), "0", "") STR(pin) ))) \
        = DEFIO_IO_REC_INITIALIZER;                                     \
    /**/

// emit usage map - bitmask of used pind for each port
#define DEFIO_IO_DEFINE_MASK()                                          \
    uint16_t ioDefMask[] = {                                          \
        DEFIO_APPLY_PORTS(DEFIO_IO_DEFINE_MASK_PORT_B,DEFIO_IO_DEFINE_MASK_PIN,DEFIO_IO_DEFINE_MASK_PORT_E) \
    }                                                                   \
    /**/

#define DEFIO_IO_DEFINE_MASK_PORT_B(port)            0
#define DEFIO_IO_DEFINE_MASK_PIN(port, pin) | (1 << pin)
#define DEFIO_IO_DEFINE_MASK_PORT_E(port)            ,

// declare ioRec_t for all available ports
#define DEFIO_IO_DECLARE()                                              \
    DEFIO_APPLY_PORTS(BOOST_PP_TUPLE_EAT(), DEFIO_IO_DECLARE_I, BOOST_PP_TUPLE_EAT()) \
    struct dummy                                                        \
    /**/

#define DEFIO_IO_DECLARE_I(port, pin)                                   \
    extern ioRec_t DEFIO_IO_REC_NAME(port, pin);                        \
    /**/


// apply op(port,pin) on all target ports
// expands to: port_b(A) pin(A,1) pin(A,2) port_e(A) port_b(B) pin(B,4) ...
#define DEFIO_APPLY_PORTS(port_b, pin, port_e)                          \
    BOOST_PP_SEQ_FOR_EACH(DEFIO_APPLY_PORTS_I, (port_b, pin, port_e), (A)(B)(C)(D)(E)(F)) \
    /**/

#define DEFIO_APPLY_PORTS_I(r, data, elem)                      \
    BOOST_PP_TUPLE_ELEM(0, data)(elem)                          \
    DEFIO_APPLY_PORT(BOOST_PP_TUPLE_ELEM(1, data), elem)        \
    BOOST_PP_TUPLE_ELEM(2, data)(elem)                          \
    /**/

// apply op(port,pin) on single port (call op for each pin)
#define DEFIO_APPLY_PORT(op, port)                                      \
    BOOST_PP_IIF(BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(TARGET_IO_PORT, port)), \
                 DEFIO_APPLY_PORT_I,                                    \
                 BOOST_PP_TUPLE_EAT(2))(op, port)                       \
    /**/

// expanded only if port is not empty
// BOOST_PP_SEQ_FOR_EACH is already used, use _I version as workaround
#define DEFIO_APPLY_PORT_I(op, port)                                    \
    BOOST_PP_SEQ_FOR_EACH_I(DEFIO_APPLY_PIN_I, (op, port), BOOST_PP_TUPLE_TO_SEQ(BOOST_PP_CAT(TARGET_IO_PORT, port))) \
    /**/

#define DEFIO_APPLY_PIN_I(r, op_port, i, pin)                           \
    BOOST_PP_TUPLE_ELEM(0, op_port)(BOOST_PP_TUPLE_ELEM(1, op_port), pin) \
    /**/

