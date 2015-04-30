#pragma once

#ifdef USE_USART1
#define USART1_RX_IO &IO_PA10
#define USART1_TX_IO &IO_PA9
#define USART1_RX_IO_REMAP &IO_PB7
#define USART1_TX_IO_REMAP &IO_PB6
#endif

#ifdef USE_USART2
#define USART2_RX_IO &IO_PA3
#define USART2_TX_IO &IO_PA2
#endif
