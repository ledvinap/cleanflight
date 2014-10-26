#pragma once

// device specific uart implementation is defined here
extern const struct serialPortVTable uartVTable;

void uartIrqHandler(uartPort_t *s);
void uartStartTxDMA(uartPort_t *s);

#ifdef USE_USART1
uartPort_t *serialUSART1(const serialPortConfig_t *config);
#endif
#ifdef USE_USART2
uartPort_t *serialUSART2(const serialPortConfig_t *config);
#endif
#ifdef USE_USART3
uartPort_t *serialUSART3(const serialPortConfig_t *config);
#endif


