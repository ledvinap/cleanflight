#define BOOST_PP_VARIADICS 1

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

// return definition tuple for given type tag
// macro CDEF_TYPE_INFO__<type> must be defined for each type
// macro CDEF_TYPE_INFO_DEFINED_<type> must be defined as comma (,)
#define CDEF_TYPE_INFO_TUPLE(typetag) BOOST_PP_ASSERT(BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(CDEF_TYPE_INFO__, typetag))) BOOST_PP_CAT(CDEF_TYPE_INFO__, typetag)
// return C type
#define CDEF_TYPE_CDECL(typetag) BOOST_PP_TUPLE_ELEM(0, CDEF_TYPE_INFO_TUPLE(typetag))
// return C type enum
#define CDEF_TYPE_METATAG(typetag) BOOST_PP_CAT(CDEF_TAG_, typetag)

// name return name from structure definition
#define CDEF_NAME(def) BOOST_PP_TUPLE_ELEM(0, def)
// return sequence with structure members
#define CDEF_MEMBER_OFFSET 1
#define CDEF_MEMBER_SEQ(def) BOOST_PP_SEQ_REST_N(CDEF_MEMBER_OFFSET, BOOST_PP_TUPLE_TO_SEQ(def))
// member name
#define CDEF_MEMBER_NAME(mdef) BOOST_PP_TUPLE_ELEM(0, mdef)
// member type tag
#define CDEF_MEMBER_TYPE(mdef) BOOST_PP_TUPLE_ELEM(1, mdef)
// member properties list
#define CDEF_MEMBER_PROPERTIES_OFFSET 2
#define CDEF_MEMBER_PROPERTIES_SEQ(mdef) BOOST_PP_SEQ_REST_N(CDEF_MEMBER_PROPERTIES_OFFSET, BOOST_PP_TUPLE_TO_SEQ(mdef))

// return structure declaration
#define CDEF_STRUCT_DECLARE(def)                                        \
    typedef struct BOOST_PP_CAT(CDEF_NAME(def), _s) {                   \
        BOOST_PP_SEQ_FOR_EACH(CDEF_STRUCT_DECLARE_MEMBER, def, CDEF_MEMBER_SEQ(def)) \
    } BOOST_PP_CAT(CDEF_NAME(def), _t)                                  \
    /**/

// declaration of one member variable
#define CDEF_STRUCT_DECLARE_MEMBER(r, _, mdef)                          \
    CDEF_TYPE_CDECL(CDEF_MEMBER_TYPE(mdef)) CDEF_MEMBER_NAME(mdef);

// return metainfo definition for fiven structure (chain of metainfo item entries)
#define CDEF_METAINFO(def)                                              \
    struct BOOST_PP_CAT(CDEF_NAME(def), _metainfo_t) {                  \
        struct cdef_metainfo_header_t header;                           \
        BOOST_PP_SEQ_FOR_EACH(CDEF_METAINFO_MEMBER_DECL, def, CDEF_MEMBER_SEQ(def)); \
    } =  {                                                              \
        CDEF_METAINFO_HEADER_VAL(def)                                   \
        BOOST_PP_SEQ_FOR_EACH(CDEF_METAINFO_MEMBER_VAL, def, CDEF_MEMBER_SEQ(def)) \
    };                                                                  \
    /**/

#define CDEF_METAINFO_HEADER_DECL(def)          \
    struct cdef_metainfo_header_t header;       \
    /**/

#define CDEF_METAINFO_HEADER_VAL(def)                           \
    .header = { .name = BOOST_PP_STRINGIZE(CDEF_NAME(def)) },   \
    /**/

#define CDEF_METAINFO_MEMBER_DECL(r, def, mdef)                         \
    struct {                                                            \
        struct  cdef_metainfo_fieldheader_t header;                     \
        CDEF_METAINFO_MEMBER_D_DECL(mdef, DEFAULT, CDEF_TYPE_CDECL(CDEF_MEMBER_TYPE(mdef)), default) \
        CDEF_METAINFO_MEMBER_D_DECL(mdef, MIN, CDEF_TYPE_CDECL(CDEF_MEMBER_TYPE(mdef)), min) \
    } BOOST_PP_CAT(f_, CDEF_MEMBER_NAME(mdef));                         \
    /**/

#define CDEF_METAINFO_MEMBER_D_DECL(mdef, tag, type, name)              \
    BOOST_PP_EXPR_IF(CDEF_PROPERTIES_TAGCOUNT(CDEF_MEMBER_PROPERTIES_SEQ(mdef), tag), \
                     type name;)                                        \
    /**/

#define CDEF_METAINFO_MEMBER_VAL(r, def, mdef)  \
    BOOST_PP_CAT(.f_, CDEF_MEMBER_NAME(mdef)) = {                       \
        .header = { .type = CDEF_TYPE_METATAG(CDEF_MEMBER_TYPE(mdef)),  \
                    .name = BOOST_PP_STRINGIZE(CDEF_MEMBER_NAME(mdef)), \
                    .offset = offsetof(struct BOOST_PP_CAT(CDEF_NAME(def), _s), CDEF_MEMBER_NAME(mdef)), \
                    .flags = 0                                          \
                    CDEF_METAINFO_MEMBER_V_FLAG(mdef, DEFAULT, DEF_DEFAULT) \
                    CDEF_METAINFO_MEMBER_V_FLAG(mdef, MIN, DEF_MIN)       \
        },                                                              \
        CDEF_METAINFO_MEMBER_V_VAL(mdef, DEFAULT, .default)             \
        CDEF_METAINFO_MEMBER_V_VAL(mdef, MIN, .min)                       \
    },                                                                  \
    /**/

#define CDEF_METAINFO_MEMBER_V_FLAG(mdef, tag, flag)                      \
    BOOST_PP_EXPR_IF(CDEF_PROPERTIES_TAGCOUNT(CDEF_MEMBER_PROPERTIES_SEQ(mdef), tag), \
                     | flag)                                            \
    /**/

#define CDEF_METAINFO_MEMBER_V_VAL(mdef, tag, fname)                    \
    BOOST_PP_IF(CDEF_PROPERTIES_TAGCOUNT(CDEF_MEMBER_PROPERTIES_SEQ(mdef), tag), \
                fname = CDEF_PROPERTIES_TAGFIRST(CDEF_MEMBER_PROPERTIES_SEQ(mdef), tag)BOOST_PP_COMMA, \
                BOOST_PP_EMPTY)()                                       \
    /**/




// numeric tags for used properties. Used to implement comparison etc.
#define CDEF_PROPERTIES_TAG__DEFAULT(...) 1
#define CDEF_PROPERTIES_TAG__DOC(...) 2
#define CDEF_PROPERTIES_TAG__COND(...) 3
#define CDEF_PROPERTIES_TAG__MIN(...) 4
#define CDEF_PROPERTIES_TAG__MAX(...) 5

// APPLY needs to be defined unless some trick to convert <tag>(x) to (x) exists
#define CDEF_PROPERTIES_APPLY__DOC(doc) (doc)
#define CDEF_PROPERTIES_APPLY__DEFAULT(val) (val)
#define CDEF_PROPERTIES_APPLY__MIN(val) (val)

    /**/

// return sequence of results for selected tag after expand and apply
#define CDEF_PROPERTIES_TAG(seq, tag)                                   \
    CDEF_PROPERTIES_EXPAND(seq,tag)                                     \
    /**/

#define CDEF_PROPERTIES_TAGCOUNT(seq, tag)                  \
    BOOST_PP_SEQ_SIZE(CDEF_PROPERTIES_TAG(seq, tag))        \
    /**/

#define CDEF_PROPERTIES_TAGFIRST(seq, tag)                          \
    BOOST_PP_SEQ_HEAD(CDEF_PROPERTIES_TAG(seq, tag)(/*default*/))   \
    /**/

#define CDEF_PROPERTIES_TAGIF(seq, tag, ifexists)                       \
    BOOST_PP_EXPR_IF(CDEF_PROPERTIES_TAGCOUNT(seq, tag), ifexists)      \
    /**/

#define CDEF_PROPERTIES_APPLY(d, tag, elem) CDEF_PROPERTIES_APPLY_I(d, tag, elem) 
#define CDEF_PROPERTIES_APPLY_I(d, tag, elem) CDEF_PROPERTIES_APPLY__ ## elem

// elem is <tag>(params), data is <tag>
#define CDEF_PROPERTIES_TAG_EQUAL_P(d, data, elem)                      \
    BOOST_PP_EQUAL(                                                     \
        BOOST_PP_CAT(CDEF_PROPERTIES_TAG__, elem),                      \
        BOOST_PP_CAT(CDEF_PROPERTIES_TAG__, data)() )                   \
    /**/

// expand all expandable properties in given sequence, then filter by tag and apply result
// while state tuple is (tag, seq, res)
#define CDEF_PROPERTIES_EXPAND(seq, tag)                                \
    BOOST_PP_TUPLE_ELEM(2, BOOST_PP_WHILE(CDEF_PROPERTIES_EXPAND_P,     \
                                          CDEF_PROPERTIES_EXPAND_O,     \
                                          (tag, seq, /*empty*/)))       \
    /**/

// predicate, repeat until input sequence is empty
#define CDEF_PROPERTIES_EXPAND_P(d, state)              \
    BOOST_PP_SEQ_SIZE(BOOST_PP_TUPLE_ELEM(1, state))    \
    /**/

// operation
// expand or apply head of input sequence
// expand test check for parens or empty expansion
#define CDEF_PROPERTIES_EXPAND_O(d, state)                              \
    BOOST_PP_IIF(BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(CDEF_PROPERTIES_EXPAND_EVAL__, BOOST_PP_SEQ_HEAD(BOOST_PP_TUPLE_ELEM(1, state))) (/*DUMMY*/)), \
                 CDEF_PROPERTIES_EXPAND_O_EXPAND,                       \
                 CDEF_PROPERTIES_EXPAND_O_TESTAPPLY                     \
        )(d, BOOST_PP_TUPLE_ELEM(0, state), BOOST_PP_TUPLE_ELEM(1, state), BOOST_PP_TUPLE_ELEM(2, state)) \
    /**/

// replace head of input seq with its expansion
// retrun state tuple (tag, seq, res)
#define CDEF_PROPERTIES_EXPAND_O_EXPAND(d, tag, seq, res)               \
    (tag, CDEF_PROPERTIES_EXPAND_EVAL(BOOST_PP_SEQ_HEAD(seq)) BOOST_PP_SEQ_TAIL(seq), res) \
    /**/

// take one elemment from input seq, check if it is tag we are interrested int, append result of apply to res
// retrun state tuple (tag, seq, res)
#define CDEF_PROPERTIES_EXPAND_O_TESTAPPLY(d, tag, seq, res)            \
    (tag,                                                               \
     BOOST_PP_SEQ_TAIL(seq),                                            \
     res BOOST_PP_IIF(CDEF_PROPERTIES_TAG_EQUAL_P(d, tag, BOOST_PP_SEQ_HEAD(seq)), \
                      CDEF_PROPERTIES_APPLY,                            \
                      BOOST_PP_TUPLE_EAT()                              \
         )(d, tag, BOOST_PP_SEQ_HEAD(seq))                              \
    )                                                                   \
    /**/

// return sequence of expanded PROPERTIES
// must be (possibly empty) sequence
#define CDEF_PROPERTIES_EXPAND_EVAL(tag_param) CDEF_PROPERTIES_EXPAND_EVAL_I(tag_param)
#define CDEF_PROPERTIES_EXPAND_EVAL_I(tag_param) CDEF_PROPERTIES_EXPAND_EVAL__ ## tag_param

// expects COND(test, property[,property])
#define CDEF_PROPERTIES_EXPAND_EVAL__COND(cond, ...)                    \
    BOOST_PP_IIF(CDEF_COND(cond), BOOST_PP_VARIADIC_TO_SEQ, BOOST_PP_TUPLE_EAT())(__VA_ARGS__) \
    /**/

// return 1 if CDEF_COND__<tag> is defined as 1
#define CDEF_COND(tag) BOOST_PP_IS_BEGIN_PARENS(BOOST_PP_CAT(CDEF_COND_IS__, BOOST_PP_CAT(CDEF_COND__,tag)))
#define CDEF_COND_IS__1 ()



#define CDEF_TYPE_INFO__UINT16 (uint16_t)

#define CONFIG_ESC_SERVO (                                              \
        escAndServoConfig,                                              \
        (minthrottle, UINT16, COND(ALIENWII32, DEFAULT(1000)), DEFAULT(1150), MIN(0), \
         DOC("Set the minimum throttle command sent to the ESC. This is the minimum value that allows motors to run at a idle speed.") ), \
        (maxthrottle, UINT16, COND(ALIENWII32, DEFAULT(200)), DEFAULT(1850), \
         DOC("Maximum value for the ESCs at full power") ),             \
        (mincommand,  UINT16, DEFAULT(1000),                             \
         DOC("Value for the ESCs when they are not armed") )           \
        )                                                               \
    /**/



#define CDEF_COND__ALIENWII32 1
#define CDEF_COND__TRUE       1
#define CDEF_COND__FALSE      0


#define d CONFIG_ESC_SERVO
#define m (minthrottle, UINT16, COND(ALIENWII32, DEFAULT(1000), COND(ALIENWII32, DEFAULT(2000))), DEFAULT(1150), \
         DOC("Set the minimum throttle command sent to the ESC. This is the minimum value that allows motors to run at a idle speed.") )

#define m2 (minthrottle, UINT16, DEFAULT(1150), COND(ALIENWII32, DEFAULT(1000)), DEFAULT(1150))

#define seq (a)

//BOOST_PP_SEQ_TAIL(seq)

CDEF_METAINFO(d)

CDEF_PROPERTIES_TAGFIRST(CDEF_MEMBER_PROPERTIES_SEQ(m), DOC)
//CDEF_PROPERTIES_EXPAND(CDEF_MEMBER_PROPERTIES_SEQ(m), DEFAULT)
//CDEF_MEMBER_PROPERTIES_SEQ(m2)
CDEF_STRUCT_DECLARE(CONFIG_ESC_SERVO)
//CDEF_MEMBER_PROPERTIES_SEQ(m)

//CDEF_PROPERTIES_FILTERAPPLY_M(1, DEFAULT, DEFAULT(1150))
//CDEF_METAINFO_MEMBER(r,d,m)
