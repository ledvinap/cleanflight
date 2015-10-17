#pragma once

bool isSerialTransmitBufferEmpty_Generic(serialPort_t *instance);
int serialTxBytesFree_Generic(serialPort_t *instance);
int serialRxBytesWaiting_Generic(serialPort_t *instance);
