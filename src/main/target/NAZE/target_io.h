#pragma once

#ifdef USE_USART1
#define USART1_RX_IO PA10
#define USART1_TX_IO PA9
#define USART1_RX_IO_REMAP PB7
#define USART1_TX_IO_REMAP PB6
#endif

#ifdef USE_USART2
#define USART2_RX_IO PA3
#define USART2_TX_IO PA2
#endif
