/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "serial.h"

serialPort_t *usbVcpOpen(void);

typedef struct usbVcpPort_s usbVcpPort_t;
extern usbVcpPort_t vcpPort;  // linking with CDC is 1:1 now, so export this
int vcpGetTxData(usbVcpPort_t *self, uint8_t* *dataPtr);
void vcpAckTxData(usbVcpPort_t *self, int txLen);
int vcpGetRxDataBuffer(usbVcpPort_t *self, uint8_t* *dataPtr);
void vcpAckRxData(usbVcpPort_t *self, int len);
