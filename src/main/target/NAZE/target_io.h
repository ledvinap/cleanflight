#pragma once

typedef enum {
    IO_NONE = 0,
    IO1, IO2, IO3, IO4, IO5, IO6, IO7, IO8, IO9, IO120, IO11, IO12, IO13, IO14, IO15, IO16
} IOId_t;

#ifdef USE_USART1
#define USART1_RX_IO IO16
#define USART1_TX_IO IO15
#define USART1_RX_IO_REMAP IO12
#define USART1_TX_IO_REMAP IO11
#endif

#ifdef USE_USART2
#define USART2_RX_IO IO4
#define USART2_TX_IO IO3
#define USART2_RX_IO_REMAP IO_NONE
#define USART2_TX_IO_REMAP IO_NONE
#endif

