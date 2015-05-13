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
#include "usb_regs.h"
#include "usb_mem.h"

#include "common/utils.h"
#include "common/atomic.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/nvic.h"

#include "serial.h"
#include "serial_usb_vcp.h"


#define USB_TIMEOUT  50
#define VCP_BUFFER_SIZE 256

static usbVcpPort_t vcpPort;
extern const struct serialPortVTable usbVcpVTable;

static void vcpTryTx(usbVcpPort_t* self);
static void vcpRx(usbVcpPort_t* self);

void usbVcpUpdateState(serialPort_t *instance, portState_t andMask, portState_t orMask)
{
    UNUSED(instance); UNUSED(andMask); UNUSED(orMask);
}


void usbVcpConfigure(serialPort_t *instance, const serialPortMode_t *config)
{
    usbVcpPort_t *self = container_of(instance, usbVcpPort_t, port);
    self->port.rxCallback = config->rxCallback;
}

void usbVcpRelease(serialPort_t *instance)
{
    UNUSED(instance);
}

void usbVcpGetConfig(serialPort_t *instance, serialPortMode_t* config)
{
    usbVcpPort_t *self = container_of(instance, usbVcpPort_t, port);
    config->rxCallback = self->port.rxCallback;
}

bool isUsbVcpTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

void usbVcpWrite(serialPort_t *instance, uint8_t ch)
{
    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    usbVcpPort_t *self = container_of(instance, usbVcpPort_t, port);

    uint16_t nxt = (self->port.txBufferHead + 1 >= self->port.txBufferSize) ? 0 : self->port.txBufferHead + 1;
    if(nxt == self->port.txBufferTail) {
        // buffer full. Discard byte now
    } else {
        self->port.txBuffer[self->port.txBufferHead] = ch;
        self->port.txBufferHead = nxt;
        ATOMIC_BLOCK_NB(NVIC_PRIO_USB) {
            ATOMIC_BARRIER(*self);
            vcpTryTx(self);
        }
    }
}

int usbVcpTotalBytesWaiting(serialPort_t *instance)
{
    int ret = instance->rxBufferHead - instance->rxBufferTail;
    if(ret < 0)
        ret += instance->rxBufferSize;
    return ret;
}

int usbVcpRead(serialPort_t *instance)
{
    usbVcpPort_t *self = container_of(instance, usbVcpPort_t, port);
    if (self->port.rxBufferHead == self->port.rxBufferTail) {
        return -1;
    }

    uint8_t ch = self->port.rxBuffer[self->port.rxBufferTail];
    self->port.rxBufferTail = (self->port.rxBufferTail + 1 >= self->port.rxBufferSize) ? 0 : self->port.rxBufferTail + 1;
    return ch;
}

serialPort_t *usbVcpOpen(void)
{
    usbVcpPort_t *self = &vcpPort;

    self->port.vTable = &usbVcpVTable;

    self->port.rxBuffer = self->rxBuffer;
    self->port.rxBufferSize = VCP_BUFFER_SIZE;
    self->port.rxBufferTail = 0;
    self->port.rxBufferHead = 0;

    self->port.txBuffer = self->txBuffer;
    self->port.txBufferSize = VCP_BUFFER_SIZE;
    self->port.txBufferTail = 0;
    self->port.txBufferHead = 0;

    self->txPending = false;

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    return &self->port;
}

// this function must be called on USB basepri
// it is not reentrant
static void vcpTryTx(usbVcpPort_t* self) {
    if(!self->txPending) {
        // transmit endpoint is empty, send new data
        int txLen;
        if (self->port.txBufferHead >= self->port.txBufferTail) {
            txLen = self->port.txBufferHead - self->port.txBufferTail;
        } else {
            txLen = self->port.txBufferSize - self->port.txBufferTail;
        }
        if(txLen == 0)
            return;  // nothing to send now
        // We can only put 64 bytes in the buffer ( /2 is copied from pervious code, check it?)

        self->txPending = true;
#if EMU_FTDI
        txLen = MIN(txLen, 64 - 2  / 2);
        uint8_t status[2] = {0x01, 0x60};
        UserToPMABufferCopy(status, ENDP1_TXADDR, 2);
        UserToPMABufferCopy(self->port.txBuffer + self->port.txBufferTail, ENDP1_TXADDR+2, txLen);
        SetEPTxCount(ENDP1, txLen + 2);
#else
        txLen = MIN(txLen, 64 / 2);
        UserToPMABufferCopy(self->port.txBuffer + self->port.txBufferTail, ENDP1_TXADDR, txLen);
        SetEPTxCount(ENDP1, txLen);
#endif
        SetEPTxValid(ENDP1);

        unsigned nxt = self->port.txBufferTail + txLen;
        if(nxt >= self->port.txBufferSize)
            nxt = 0;
        self->port.txBufferTail = nxt;
    }
}

// this function must be called on USB basepri
static void vcpRx(usbVcpPort_t* self)
{
#ifdef EMU_FTDI
    int rxLen = GetEPRxCount(ENDP2);
    int rxOfs = ENDP2_RXADDR;
#else
    int rxLen = GetEPRxCount(ENDP3);
    int rxOfs = ENDP3_RXADDR;
#endif
    while(rxLen > 0) {
        int rxChunk;
        if (self->port.rxBufferHead >= self->port.rxBufferTail) {
            // space up to end of buffer
            rxChunk = self->port.rxBufferSize - self->port.rxBufferHead;
        } else {
            // space to tail - 1
            rxChunk = self->port.rxBufferTail - 1 - self->port.rxBufferHead;
        }
        if(!rxChunk) {
            // receive bufer if full
            break;
        }
        rxChunk = MIN(rxChunk, rxLen);
        PMAToUserBufferCopy(self->port.rxBuffer + self->port.rxBufferHead, rxOfs, rxChunk);
        rxLen -= rxChunk; rxOfs += rxChunk;
        unsigned nxt = self->port.rxBufferHead + rxChunk;
        if(nxt >= self->port.rxBufferSize)
            nxt = 0;
        self->port.rxBufferHead = nxt;
    }
#ifdef EMU_FTDI
    SetEPRxCount(ENDP2, 64);
    SetEPRxStatus(ENDP2, EP_RX_VALID);
#else
    SetEPRxCount(ENDP3, 64);
    SetEPRxStatus(ENDP3, EP_RX_VALID);
#endif
}

// EP1 in callback - transmission finished
void EP1_IN_Callback(void)
{
    vcpPort.txPending = false;
    vcpTryTx(&vcpPort);
}

#ifdef EMU_FTDI
// EP2 out callback - data received
void EP2_OUT_Callback(void)
{
    vcpRx(&vcpPort);
}
#else
// EP3 out callback - data received
void EP3_OUT_Callback(void)
{
    vcpRx(&vcpPort);
}
#endif

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

