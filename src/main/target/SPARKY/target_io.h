#pragma once

// available IO pins
// TARGET_IO_PORTx is bitmask of used ping for each port
// unused ports does not need to be defined
#define TARGET_IO_PORTA 0xffff  // TODO!
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(6))
