#pragma once

int16_t autotunePIDRelay(void);

bool isAutotuneIdle(void);
void autotuneReset(void);
void autotuneActivated(void);
void autotuneDeactivated(void);
bool autotuneShouldSavePIDs(void);
