
#pragma once

#include <stdbool.h>

void bbmspWriteByte(uint8_t byte);
int bbmspWrite(const uint8_t *data, unsigned len);
bool bbmspFlush(void);
void bbmspInfo(int cmd);
