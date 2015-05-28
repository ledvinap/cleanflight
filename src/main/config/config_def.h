#define BOOST_PP_VARIADICS 1

#include <boost/preprocessor/arithmetic/dec.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/list/filter.hpp>
#include <boost/preprocessor/list/for_each.hpp>
#include <boost/preprocessor/list/rest_n.hpp>
#include <boost/preprocessor/logical/or.hpp>
#include <boost/preprocessor/seq.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/tuple.hpp>
#include <boost/preprocessor/variadic.hpp>
#include <boost/preprocessor/debug/assert.hpp>
#include <boost/preprocessor/punctuation.hpp>
#include <boost/preprocessor/punctuation/is_begin_parens.hpp>

// name return name from structure definition
#define CDEF_NAME(def) BOOST_PP_TUPLE_ELEM(0, def)
// return sequence with structure members (mdef)
#define CDEF_MEMBER_OFFSET 1
#define CDEF_MEMBER_SEQ(def) BOOST_PP_SEQ_REST_N(CDEF_MEMBER_OFFSET, BOOST_PP_TUPLE_TO_SEQ(def))
// member name
#define CDEF_MEMBER_DEF(mdef) BOOST_PP_TUPLE_ELEM(0, mdef)

// return structure declaration
#define CDEF_STRUCT(def) struct BOOST_PP_CAT(CDEF_NAME(def), _s)
#define CDEF_STRUCT_DECLARE(def)                                        \
    CDEF_STRUCT(def) {                                                  \
        BOOST_PP_SEQ_FOR_EACH(CDEF_STRUCT_DECLARE_MEMBER, def, CDEF_MEMBER_SEQ(def)) \
    } BOOST_PP_CAT(CDEF_NAME(def), _t)                                  \
    /**/

// declaration of one member variable
#define CDEF_STRUCT_DECLARE_MEMBER(r, _, mdef)  \
    CDEF_MEMBER_DEF(mdef);

#define CDEF_METAINFO_REF(def) (const union cdef_metainfo_typedef *)&BOOST_PP_CAT(CDEF_NAME(def), _metainfo)


#include <stdint.h>
#include "common/utils.h"

#define member_size(type, member) sizeof(((type *)0)->member)

#define PACKED __attribute__ ((packed))

#define MF_TYPE_OFS               0
#define MF_TYPE_SZE            2
#define MF_TYPE_FIELD       cdef_field
#define MF_TYPE_ARRAY       cdef_array
#define MF_TYPE_PRIMITIVE   cdef_primitive

#define MF_DEFSIZE          (1 <<  2)
#define MF_DATSIZE          (1 <<  3)
#define MF_LAST             (1 <<  4)
#define MF_NAMEPTR          (1 <<  5)
#define MF_DATAOFFSET       (1 <<  6)
#define MF_ARRAY_STRIDE     (1 <<  7)
#define MF_ARRAY_COUNT      (1 <<  8)

#define MF_SUBDATA_OFS           11
#define MF_SUBDATA_SZE         2
#define MF_SUBDATA_REF       0
#define MF_SUBDATA_DIRECT    1
#define MF_SUBDATA_PRIMITIVE 2

#define MF_DFLT             (1 << 13)
#define MF_MIN              (1 << 14)
#define MF_MAX              (1 << 15)


// start of something that we can point to
// structure/array/functional ref/...
enum cdef_metainfo_type {
    cdef_field,
    cdef_array,
    cdef_primitive,
};

enum cdef_primitiveType {
    cdef_primitive_NONE,
    cdef_primitive_UINT16,
} PACKED;

union cdef_metainfo_typedef {                  // all possible attached data types, flags determine what is included, must be in order defined here, some may be repeated
    uint16_t flags;             // type + type specific flags, always present
    uint16_t defSize;           // size of this type record. Will be calculated when missing
    uint16_t datSize;           // size of described data type. Will be calculated when missing
    const char* namePtr;
    int16_t dataOffset;
    int16_t arrayStride;
    uint16_t arrayCount;
    // subtype related info
    enum cdef_primitiveType primitiveType;
    const union cdef_metainfo_typedef *defPtr;
    char subDef[0];
    // tag related data. Must be placed after subtype info (data size needed)
    char dflt[0];
    char min[0];
    char max[0];
} PACKED;
