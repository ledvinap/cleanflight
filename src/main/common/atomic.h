#pragma once

// cleanup restore function, with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    __asm__ volatile ("" ::: "memory");
    __set_BASEPRI(*val);
}

// set basepri function, with global memory barrier
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI(prio);
    __asm__ volatile ("" ::: "memory");
    return 1;
}

// cleanup restore function, no memory barrier
static inline void __basepriRestore(uint8_t *val)
{
    __set_BASEPRI(*val);
}

// set basepri function, no memory
static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
    __set_BASEPRI(prio);
    return 1;
}

// Run block with elevated BASEPRI, restoring BASEPRI on exit. Full memory barrier is place at start and end of block
#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

// Run block with elevated BASEPRI, but do not create any memory barrier.
#define ATOMIC_BLOCK_NB(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestore))) = __get_BASEPRI(), \
                                    __ToDo = __basepriSetRetVal(prio); __ToDo ; __ToDo = 0 ) \

// Create memory barrier at the beginning (all data must be reread from structure)
// and end of block (all data must be written, but may be cached in register for subsequent use)
// ideally this would only protect stucture passed as parameter, but gcc is curently creating almost full barrier
//   check resulting assembly. Typing to volatile pointer is probably better in time-critical sections
// this macro can be used only ONCE PER LINE, but multiple uses per block should be fine

#ifndef __UNIQL
# define __UNIQL_CONCAT2(x,y) x ## y
# define __UNIQL_CONCAT(x,y) __UNIQL_CONCAT2(x,y)
# define __UNIQL(x) __UNIQL_CONCAT(x,__LINE__)
#endif


#define ATOMIC_BARRIER(data)                                            \
    __extension__ void  __UNIQL(__barrierEnd)(typeof(data) **__d) {     \
        __asm__ volatile ("" : : "m" (**__d));                          \
    }                                                                   \
    typeof(data)  __attribute__((__cleanup__(__UNIQL(__barrierEnd)))) *__UNIQL(__barrier) = &data; \
    __asm__ volatile ("" : "=m" (*__UNIQL(__barrier)))

// This is just palceholder to mark protected structures where other (optimized) synchronization is used
#define ATOMIC_BARRIER_DUMMY(data)

#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
