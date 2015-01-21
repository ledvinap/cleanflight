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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "usb_core.h"
#include "usb_init.h"
#include "vcp/hw_config.h"

#include "drivers/system.h"
#include "common/utils.h"

#include "serial.h"
#include "serial_usb_vcp.h"


#define USB_TIMEOUT  50

static usbVcpPort_t vcpPort;
extern const struct serialPortVTable usbVcpVTable;

void usbVcpUpdateState(serialPort_t *serial, portState_t andMask, portState_t orMask)
{
    UNUSED(instance);
    UNUSED(baudRate);

    // TODO implement
}

void usbVcpConfigure(serialPort_t *serial, const serialPortConfig_t *config)
{
    UNUSED(instance);
    UNUSED(mode);

    // TODO implement
}

void usbVcpRelease(serialPort_t *serial)
{
    // TODO implement
}

void usbVcpGetConfig(serialPort_t *serial, serialPortConfig_t* config)
{
    usbVcpPort_t *self = container_of(serial, usbVcpPort_t, port);

    config->baudRate = self->port.baudRate;  // TODO - use actual baudrate
    config->mode = self->port.mode;
    config->rxCallback = self->port.rxCallback;
}

bool isUsbVcpTransmitBufferEmpty(serialPort_t *serial)
{
    UNUSED(serial);
    return true;
}

void usbVcpWrite(serialPort_t *serial, uint8_t c)
{
    UNUSED(serial);
    uint32_t txed;
    uint32_t start = millis();

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    do {
        txed = CDC_Send_DATA((uint8_t*)&c, 1);
    } while (txed < 1 && (millis() - start < USB_TIMEOUT));
}

int usbVcpTotalBytesWaiting(serialPort_t *serial)
{
    UNUSED(serial);
    return receiveLength;
}

int usbVcpRead(serialPort_t *serial)
{
    UNUSED(serial);
    uint8_t buf[1];

    uint32_t rxed = 0;

    while (rxed < 1) {
        rxed += CDC_Receive_DATA((uint8_t*)buf + rxed, 1 - rxed);
    }

    return buf[0];
}

serialPort_t *usbVcpOpen(void)
{
    usbVcpPort_t *s;

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    s = &vcpPort;
    s->port.vTable = &usbVcpVTable;

    return &s->port;
}

const struct serialPortVTable usbVcpVTable = {
    .isTransmitBufferEmpty = isUsbVcpTransmitBufferEmpty,
    .write = usbVcpWrite,
    .totalBytesWaiting = usbVcpTotalBytesWaiting,
    .read = usbVcpRead,

    .release = usbVcpRelease,
    .configure = usbVcpConfigure,
    .getConfig = usbVcpGetConfig,
    .updateState = usbVcpUpdateState
};

