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

#include <p99_args.h>

// return parameter if it is single tupple, otherwise return tupple containing arguments
#define TO_TUPLE_IDENTITY(x) x
#define TO_TUPLE(...) BOOST_PP_IIF(P99_IS_EMPTY(BOOST_PP_TUPLE_EAT() __VA_ARGS__),TO_TUPLE_IDENTITY,BOOST_PP_VARIADIC_TO_TUPLE)(__VA_ARGS__)

// return definition tupple for given type tag
// macro CDEF_TYPE_INFO__<type> must be defined for each type
#define CDEF_TYPE_INFO_TUPPLE(typetag) BOOST_PP_CAT(CDEF_TYPE_INFO__, typetag)
// return C type
#define CDEF_TYPE_CDECL(typetag) BOOST_PP_TUPLE_ELEM(0, CDEF_TYPE_INFO_TUPPLE(typetag))
// return C type enum
#define CDEF_TYPE_METATAG(typetag) BOOST_PP_CAT(cdef_tag_, typetag)

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
    } BOOST_PP_CAT(CDEF_NAME(def), _t)

// declaration of one member variable
#define CDEF_STRUCT_DECLARE_MEMBER(r, _, mdef)                          \
    CDEF_TYPE_CDECL(CDEF_MEMBER_TYPE(mdef)) CDEF_MEMBER_NAME(mdef);

// return metainfo definition for fiven structure (chain of metainfo item entries)
#define CDEF_METAINFO(def)                                              \
    union cdef_metainfo_item BOOST_PP_CAT(CDEF_NAME(def), _metainfo)[] = { \
        CDEF_METAINFO_HEADER(def)                                       \
        BOOST_PP_SEQ_FOR_EACH(CDEF_METAINFO_MEMBER, def, CDEF_MEMBER_SEQ(def)) \
    };                                                                  \
    /**/

#define CDEF_METAINFO_HEADER(def)                            \
    { .header = { } },                                       \
    { .name = BOOST_PP_STRINGIZE(CDEF_NAME(def)) },          \
    /**/

#define CDEF_METAINFO_MEMBER(r, def, mdef)                       \
    { .fieldheader = { .type = CDEF_TYPE_METATAG(CDEF_MEMBER_TYPE(mdef)) } }, \
    { .name = BOOST_PP_STRINGIZE(CDEF_MEMBER_NAME(mdef)) },             \
    { .offset = offsetof(struct BOOST_PP_CAT(CDEF_NAME(def), _s), CDEF_MEMBER_NAME(mdef)) }, \
    { .default = CDEF_PROPERTIES_GETFIRST(CDEF_MEMBER_PROPERTIES_SEQ(mdef), DEFAULT) }, \
/**/


// numeric tags for used properties. Used to implement comparison etc.
#define CDEF_PROPERTIES_TAG__DEFAULT(...) 1
#define CDEF_PROPERTIES_TAG__DOC(...) 2
#define CDEF_PROPERTIES_TAG__COND(...) 3
#define CDEF_PROPERTIES_TAG__TEST(...) 4

#define CDEF_PROPERTIES_APPLY__DOC(doc) (doc)
#define CDEF_PROPERTIES_APPLY__DEFAULT(val) (val)

    /**/

#define CDEF_PROPERTIES_GETFIRST(seq, tag)                              \
    BOOST_PP_SEQ_HEAD(BOOST_PP_SEQ_FOR_EACH(CDEF_PROPERTIES_APPLY_M,    \
                                            CDEF_PROPERTIES_APPLY__,    \
                                            BOOST_PP_SEQ_FILTER(CDEF_PROPERTIES_TAG_EQUAL_P, tag, seq) )) \
    /**/

// do no use cat here, applied macro may use it
#define CDEF_PROPERTIES_APPLY_M(d, data, elem) CDEF_PROPERTIES_APPLY_M_I(d, data, elem)
#define CDEF_PROPERTIES_APPLY_M_I(d, data, elem) data ## elem

// elem is <tag>(params), data is <tag>
#define CDEF_PROPERTIES_TAG_EQUAL_P(d, data, elem)                      \
    BOOST_PP_EQUAL(                                                     \
        BOOST_PP_CAT(CDEF_PROPERTIES_TAG__, elem),         \
        BOOST_PP_CAT(CDEF_PROPERTIES_TAG__, data)() )      \
    /**/

// expand all expandable properties in given sequence
#define CDEF_PROPERTIES_EXPAND(seq)                                     \
    BOOST_PP_TUPLE_ELEM(1, BOOST_PP_WHILE(CDEF_PROPERTIES_EXPAND_P,     \
                                          CDEF_PROPERTIES_EXPAND_O,     \
                                          (seq,BOOST_PP_SEQ_NIL)))      \
    /**/

// repeat until sequence is empty
#define CDEF_PROPERTIES_EXPAND_P(d, state) \
    BOOST_PP_SEQ_SIZE(BOOST_PP_TUPLE_ELEM(0,state))

#define CDEF_PROPERTIES_EXPAND_O(d, state) CDEF_PROPERTIES_EXPAND_O_D(d, BOOST_PP_TUPLE_ELEM(0, state), BOOST_PP_TUPLE_ELEM(1, state))
#define CDEF_PROPERTIES_EXPAND_O_D(d, seq, res)                     \
    BOOST_PP_IIF(P99_IS_EMPTY(BOOST_PP_CAT(CDEF_PROPERTIES_EXPAND_TEST_, BOOST_PP_SEQ_HEAD(seq))), \
                 (CDEF_PROPERTIES_EXPAND_EVAL(BOOST_PP_SEQ_HEAD(seq)) BOOST_PP_SEQ_TAIL(seq), \
                  res),                                                 \
                 (BOOST_PP_SEQ_TAIL(seq),                               \
                  BOOST_PP_SEQ_PUSH_BACK(res, BOOST_PP_SEQ_HEAD(seq)) ) ) \
        /**/

// list of properties that can be expanded
#define CDEF_PROPERTIES_EXPAND_TEST_COND(...) /* empty */

// return sequernce of expanded PROPERTIES
#define CDEF_PROPERTIES_EXPAND_EVAL(expand_with_param) CDEF_PROPERTIES_EXPAND_EVAL_I(expand_with_param)
#define CDEF_PROPERTIES_EXPAND_EVAL_I(expand_with_param) CDEF_PROPERTIES_EXPAND_EVAL_ ## expand_with_param

// expects COND(test, property[,property]) or (COND((test1, test2),  property[,property])
#define CDEF_PROPERTIES_EXPAND_EVAL_COND(cond_or_condtupple, ...)       \
    BOOST_PP_IIF(                                                       \
        CDEF_COND_SEQ(BOOST_PP_TUPLE_TO_SEQ(TO_TUPLE(cond_or_condtupple))), \
        BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__),                          \
        BOOST_PP_EMPTY() )                                              \
/**/

// check condition sequence - scans sequence and returns 1 if any CDEF_COND_<cond> is set
#define CDEF_COND_SEQ(seq) BOOST_PP_SEQ_FOLD_LEFT(CDEF_COND_SEQ_OP_OR, 0, seq)
#define CDEF_COND_SEQ_OP_OR(d, state, elem) BOOST_PP_OR(state, CDEF_COND(elem))

// return 1 if CDEF_COND__<tag> is defined as 1
#define CDEF_COND(tag) P99_IS_EMPTY(BOOST_PP_CAT(CDEF_COND_IS_, BOOST_PP_CAT(CDEF_COND__,tag)))
#define CDEF_COND_IS_1 /* empty */



#define CDEF_TYPE_INFO__UINT16 (uint16_t)

#define CONFIG_ESC_SERVO (                                              \
        escAndServoConfig,                                              \
        (minthrottle, UINT16, COND(ALIENWII32, DEFAULT(1000)), DEFAULT(1150), \
         DOC("Set the minimum throttle command sent to the ESC. This is the minimum value that allows motors to run at a idle speed.") ), \
        (maxthrottle, UINT16, DEFAULT(2000, ALIENWII32), DEFAULT(1850),            \
         DOC("Maximum value for the ESCs at full power") ),             \
        (mincommand,  UINT16, DEAFULT(1000),                             \
         DOC("Value for the ESCs when they are not armed") )           \
        )                                                               \
    /**/

#define m (minthrottle, UINT16, DEFAULT(1000, ALIENWII32), DEFAULT(1150),          \
           DOC("Set the minimum throttle command sent to the ESC. This is the minimum value that allows motors to run at a idle speed.") )

#define CDEF_COND__ALIENWII32 1
#define CDEF_COND__TRUE       1
#define CDEF_COND__FALSE      0

#define t (COND((FALSE, NONE), DEFAULT(1000), COND(FALSE, TEST(2))))(DEFAULT(1150))(TEST(1))
#define t2 (DEFAULT(1))(DOC(3))(TEST(2))

BOOST_PP_SEQ_FILTER(CDEF_PROPERTIES_TAG_EQUAL_P, DOC, t2) 

CDEF_PROPERTIES_TAG_EQUAL_P(1, DOC, TEST())

CDEF_PROPERTIES_GETFIRST(t2, DOC)

CDEF_PROPERTIES_EXPAND_EVAL_COND(TRUE, DEFAULT(1000), TEST(2))


#define seq (COND(TRUE, DEFAULT(1000), TEST(2)))(DEFAULT(1150))(TEST(1))

P99_PASTE2(CDEF_PROPERTIES_APPLY_COND_EVAL_, COND2(TRUE, DEFAULT(1000), TEST(2)))

CDEF_PROPERTIES_APPLY_COND_EVAL_COND(TRUE, DEFAULT(1000), TEST(2))

OOST_PP_SEQ_HEAD(seq)

 BOOST_PP_SEQ_TAIL(seq)
                  res)

CDEF_STRUCT_DECLARE(CONFIG_ESC_SERVO);
CDEF_METAINFO(CONFIG_ESC_SERVO);

CDEF_COND_SEQ((FALS))

CDEF_PROPERTIES_A_COND((FALSE,TRUE), DOC("test"), DOC("test2"))

CDEF_MEMBER_SEQ(CONFIG_ESC_SERVO)

BOOST_PP_VARIADIC_SIZE()

CDEF_STRUCT_MEMBER_PROPERTIES_GETFIRST(CDEF_MEMBER_PROPERTIES_LIST(m), DOC)

CDEF_MEMBER_PROPERTIES_LIST(m)

CDEF_CHECK_COND(BOOST_PP_VARIADIC_TO_LIST(FALSE, TRUE))

CDEF_STRUCT_MEMBER_PROPERTIES_A_DEFAULT(10000, TRUE)

CDEF_STRUCT_MEMBER_PROPERTIES_APPLY_MACRO(1, CDEF_STRUCT_MEMBER_PROPERTIES_A_, DEFAULT(1000, FALSE))

BOOST_PP_LIST_FILTER(CDEF_STRUCT_MEMBER_PROPERTIES_IS_TAG_EQUAL, DEFAULT, CDEF_MEMBER_PROPERTIES_LIST(m))

CDEF_STRUCT_MEMBER_PROPERTIES_IS_TAG_EQUAL(d, DEFAULT, DEFAULT(1000, ALIENWII32))

CDEF_MEMBER_PROPERTIES_LIST(m)


BOOST_PP_VARIADIC_SIZE(1,2)

#define S (1)
BOOST_PP_SEQ_SIZE(S)
