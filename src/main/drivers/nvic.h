
#pragma once

#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2

// can't use 0
#define NVIC_PRIO_MAX                      NVIC_BUILD_PRIORITY(0, 1)
#define NVIC_PRIO_TIMER                    NVIC_BUILD_PRIORITY(0, 1)
#define NVIC_PRIO_BARO_EXTI                NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_SONAR_EXTI               NVIC_BUILD_PRIORITY(2, 0)  // maybe increate slightly
#define NVIC_PRIO_MPU_INT_EXTI             NVIC_BUILD_PRIORITY(0x0f, 0x0f)
#define NVIC_PRIO_MAG_INT_EXTI             NVIC_BUILD_PRIORITY(0x0f, 0x0f)

#define NVIC_PRIO_WS2811_DMA               NVIC_BUILD_PRIORITY(1, 2)  // TODO - is there some reason to use high priority? (or to use DMA IRQ at all?)
#define NVIC_PRIO_TRANSPONDER_DMA          NVIC_BUILD_PRIORITY(3, 0)
#define NVIC_PRIO_SERIALUART1_TXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART1_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART1              NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART2_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART2              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART3_TXDMA        NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART3_RXDMA        NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART3              NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART4_TXDMA       NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART4_RXDMA       NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART4             NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_SERIALUART5_TXDMA       NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_SERIALUART5_RXDMA       NVIC_BUILD_PRIORITY(1, 1)
#define NVIC_PRIO_SERIALUART5             NVIC_BUILD_PRIORITY(1, 2)
#define NVIC_PRIO_I2C_ER                   NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_I2C_EV                   NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_USB                      NVIC_BUILD_PRIORITY(2, 0)
#define NVIC_PRIO_USB_WUP                  NVIC_BUILD_PRIORITY(1, 0)
#define NVIC_PRIO_TIME_TIMER               NVIC_BUILD_PRIORITY(0x0f, 0x0f)   // used to keep track of time timer overflows
#define NVIC_PRIO_SYSTICK                  NVIC_BUILD_PRIORITY(0x0f, 0x0f)   // must be same priority group as CALLBACK
#define NVIC_PRIO_CALLBACK                 NVIC_BUILD_PRIORITY(0x0f, 0x0f)

#define NVIC_PRIO_TIMER_PWMOUT             NVIC_BUILD_PRIORITY(3, 1)

// utility macros to join/split priority
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
