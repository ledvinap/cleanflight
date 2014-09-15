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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SOFT_SERIAL

#include "build_config.h"

#include "common/utils.h"

#include "system.h"
#include "gpio.h"
#include "timer.h"
#include "callback.h"

#include "pwm_mapping.h"

#include "serial.h"
#include "serial_softserial.h"

#if defined(STM32F10X_MD) || defined(CHEBUZZF3)
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 10 // PWM 5
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 11 // PWM 6
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 12 // PWM 7
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 13 // PWM 8
#endif

#if defined(STM32F303xC) && !defined(CHEBUZZF3)
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 8 // PWM 9
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 9 // PWM 10
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 10 // PWM 11
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 11 // PWM 12
#endif

#define RX_BITS 8
#define RX_TOTAL_BITS (1+RX_BITS+1)
#define TX_BITS 8
#define TX_TOTAL_BITS (1+TX_BITS+1)

#define MAX_SOFTSERIAL_PORTS 2
softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

extern const struct serialPortVTable softSerialVTable[];

void softSerialTxCallback(callbackRec_t *cb);
void softSerialRxCallback(callbackRec_t *cb);
void softSerialTryTx(softSerial_t* self);
void softSerialRxTimeoutEvent(timerQueueRec_t *tq_ref);

static void resetBuffers(softSerial_t *self)
{
    self->port.rxBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    self->port.rxBuffer = self->rxBuffer;
    self->port.rxBufferTail = 0;
    self->port.rxBufferHead = 0;

    self->port.txBuffer = self->txBuffer;
    self->port.txBufferSize = SOFT_SERIAL_BUFFER_SIZE;
    self->port.txBufferTail = 0;
    self->port.txBufferHead = 0;
}

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, 
                             uint32_t baud, uint8_t mode, serialInversion_e inversion)
{
    softSerial_t *self = &(softSerialPorts[portIndex]);

    if (portIndex == SOFTSERIAL1) {
        self->rxTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_RX_HARDWARE]);
        self->txTimerHardware = &(timerHardware[SOFT_SERIAL_1_TIMER_TX_HARDWARE]);
    }

    if (portIndex == SOFTSERIAL2) {
        self->rxTimerHardware = &(timerHardware[SOFT_SERIAL_2_TIMER_RX_HARDWARE]);
        self->txTimerHardware = &(timerHardware[SOFT_SERIAL_2_TIMER_TX_HARDWARE]);
    }

    self->port.vTable = softSerialVTable;
    self->port.baudRate = baud;
    self->port.mode = mode;
    self->port.inversion = inversion;
    self->port.callback = callback;

    resetBuffers(self);

    self->transmissionErrors = 0;
    self->receiveErrors = 0;
    
    self->bitTime=(1000000<<8)/baud;                     // fractional number of timer ticks per bit, 24.8 fixed point
    self->invBitTime=((long long)baud<<16)/1000000;      // scaled inverse bit time. Used to to get bit number (multiplication is faster)

    self->rxBitIndex=-1;    // TODO 
    self->rxInternalBuffer=0;      
    

    if(mode & MODE_SINGLEWIRE) {
        // in sinlgewire we setup RX mode here
        // alse use RX pin only
        self->txTimerHardware=self->rxTimerHardware;
        self->directionTx=false;
        callbackRegister(&self->rxCallback, softSerialRxCallback);
        callbackRegister(&self->txCallback, softSerialTxCallback);
        timerOut_Config(&self->txTimerCh, self->txTimerHardware, &self->txCallback, (inversion?TIMEROUT_INVERTED:0)|TIMEROUT_WAKEONEMPTY);
        timerOut_Release(&self->txTimerCh);
        timerIn_Config(&self->rxTimerCh, self->rxTimerHardware, &self->rxCallback, (inversion?0:TIMERIN_FLAG_HIGH)|TIMERIN_POLARITY_TOGGLE);
        timerIn_SetBuffering(&self->rxTimerCh, 0);
        timerQueue_Config(&self->rxTimerQueue, softSerialRxTimeoutEvent);
    } else {
        if(mode & MODE_RX) {
            callbackRegister(&self->rxCallback, softSerialRxCallback);
            timerIn_Config(&self->rxTimerCh, self->rxTimerHardware, &self->rxCallback, (inversion?0:TIMERIN_FLAG_HIGH)|TIMERIN_POLARITY_TOGGLE);
            timerIn_SetBuffering(&self->rxTimerCh, 0);
            timerQueue_Config(&self->rxTimerQueue, softSerialRxTimeoutEvent);
        }
        if(mode & MODE_TX) {
            callbackRegister(&self->txCallback, softSerialTxCallback);
            timerOut_Config(&self->txTimerCh, self->txTimerHardware, &self->txCallback, (inversion?TIMEROUT_INVERTED:0));
        }
    }
    return &self->port;
}

void softSerialSetDirection(serialPort_t *serial, portDirection_t direction)
{
    softSerial_t* self=container_of(serial, softSerial_t, port);
    if(direction==DIRECTION_RX) {
        timerOut_Release(&self->txTimerCh);
        self->directionTx=false;
        timerIn_Restart(&self->rxTimerCh);
    } else if(direction==DIRECTION_TX) {
        timerIn_Release(&self->rxTimerCh);
        self->directionTx=true; 
        self->directionRxOnDone=false;     
        timerOut_Restart(&self->txTimerCh);
    } else if(direction==DIRECTION_RX_WHENTXDONE) {
        self->directionRxOnDone=true;
        // TODO - set flags
    }
}

void softSerialTxCallback(callbackRec_t *cb)
{
    softSerial_t *self=container_of(cb, softSerial_t, txCallback);;
    softSerialTryTx(self);
    if(self->directionRxOnDone && timerOut_QLen(&self->txTimerCh)==0) {
        softSerialSetDirection(&self->port, DIRECTION_RX);
    }
}

void softSerialTryTx(softSerial_t* self) {
    while(!isSoftSerialTransmitBufferEmpty(&self->port)           // do we have something to send?  
          && timerOut_QSpace(&self->txTimerCh)>TX_TOTAL_BITS) {  // we need space for whole byte

    
        uint16_t byteToSend = self->port.txBuffer[self->port.txBufferTail];
        self->port.txBufferTail=(self->port.txBufferTail+1)%self->port.txBufferSize; 
    
        byteToSend<<=1;                 // add startbit
        byteToSend|=1<<(TX_BITS+1);     // add stopbit
    
        int bitPos=0;
        uint16_t lastEdgePos=0;
        while(byteToSend) {
            uint16_t flags=(byteToSend&1)?TIMEROUT_FLAG_ACTIVE:0;
            int bitcnt=(byteToSend&1)?__builtin_ctz(~byteToSend):__builtin_ctz(byteToSend);
            byteToSend>>=bitcnt;
            bitPos+=bitcnt;
            uint16_t edgePos=(bitPos*self->bitTime)>>8;
            if(!byteToSend)
                flags|=TIMEROUT_FLAG_WAKE;  // set wake flag for last interval
            timerOut_QPush(&self->txTimerCh, edgePos-lastEdgePos, flags);
            lastEdgePos=edgePos;
        }
        timerOut_QCommit(&self->txTimerCh);   // this will start transmission if necessary
    }
}

#define STOPBIT_MASK (0x8000)
void srStoreByte(softSerial_t *self) {
    // check stopbits. Startbit must be correct by desing
    if((self->rxInternalBuffer & STOPBIT_MASK) != STOPBIT_MASK) {
        self->receiveErrors++;
        return;
    }
    uint8_t byte=self->rxInternalBuffer>>(sizeof(self->rxInternalBuffer)*8-RX_TOTAL_BITS + 1); // shift startbit out!
    if (self->port.callback) {
        self->port.callback(byte);
    } else {
        self->port.rxBuffer[self->port.rxBufferHead] = byte;
        self->port.rxBufferHead = (self->port.rxBufferHead + 1) % self->port.rxBufferSize;
    }
}

void softSerialRxCallback(callbackRec_t *cb)
{
    softSerial_t *self=container_of(cb, softSerial_t, rxCallback);
    uint16_t capture, flags;
    while(timerIn_QPop(&self->rxTimerCh, &capture, &flags)) {
        if (self->rxBitIndex<0) {
            // startbit edge needs to be processed as data first. goto here to restart startbit processing.
            // maybe there is some way cheaner way to do this ... 
        check_startbit:                     
            if(flags&TIMERIN_FLAG_TIMER)    // waiting for startbit, discard timer 
                continue;
            if(self->port.inversion?(flags&TIMERIN_FLAG_HIGH):!(flags&TIMERIN_FLAG_HIGH))
                continue;                   // must be correct polarity
            self->rxStartRef = capture - ((self->bitTime / 2) >> 8);  // Some precision is lost here. Maybe scale the value first
            self->rxBitIndex = 0;
            self->rxInternalBuffer = 0x8000;                          // not neccessary, but easier to debug
            timerQueue_Start(&self->rxTimerQueue, capture-timerIn_getTimCNT(&self->rxTimerCh)+((self->bitTime*RX_TOTAL_BITS)>>8)+20);  // add some time to wait for next startbit
            timerIn_SetBuffering(&self->rxTimerCh, RX_TOTAL_BITS); 
            // TODO - setup byte timeout here. No need to process individual edges until whole byte is received
            
            continue;                                                 // try with next edge
        }

        int8_t bitIdx=((unsigned long)((capture-self->rxStartRef)&0xffff)*self->invBitTime)>>16;
        if(bitIdx>=RX_TOTAL_BITS) {
            bitIdx=RX_TOTAL_BITS;
        } else {
            // ignore timer events unless ther finish byte
            // this simplifies invalid edge detection 
            if(flags&TIMERIN_FLAG_TIMER)
                continue;
        }
        int8_t bitsReceived = bitIdx-self->rxBitIndex;
        if(bitsReceived<=0) {
            // invalid bit interval or repeated edge in same bit
            self->receiveErrors++;
            self->rxBitIndex=-1;
            goto check_startbit;
        }
        // shift in reveived bits
        self->rxInternalBuffer>>=bitsReceived; // zeroes are shifted in
        if(self->port.inversion?!(flags&TIMERIN_FLAG_HIGH):(flags&TIMERIN_FLAG_HIGH)) { 
            // set new bits to one
            self->rxInternalBuffer|=~((0xffff) >> bitsReceived); 
        }
        if(bitIdx==RX_TOTAL_BITS) {
            srStoreByte(self);
            self->rxBitIndex=-1;
            goto check_startbit;
        } else {
            self->rxBitIndex=bitIdx;
        }
    }
}

void softSerialRxTimeoutEvent(timerQueueRec_t *tq_ref)
{
    softSerial_t* self=container_of(tq_ref, softSerial_t, rxTimerQueue);
    timerIn_Flush(&self->rxTimerCh);
    timerIn_SetBuffering(&self->rxTimerCh, 0);
}

uint8_t softSerialTotalBytesWaiting(serialPort_t *instance)
{
    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }

    softSerial_t *s = (softSerial_t *)instance;

    return (s->port.rxBufferHead - s->port.rxBufferTail) % (s->port.rxBufferSize);
}

uint8_t softSerialReadByte(serialPort_t *instance)
{
    uint8_t ch;

    if ((instance->mode & MODE_RX) == 0) {
        return 0;
    }
    
    softSerial_t *s = (softSerial_t *)instance;
    if (s->port.rxBufferHead == s->port.rxBufferTail) {
        return 0;
    }

    ch = instance->rxBuffer[instance->rxBufferTail];
    instance->rxBufferTail = (instance->rxBufferTail + 1) % instance->rxBufferSize;
    return ch;
}

void softSerialWriteByte(serialPort_t *s, uint8_t ch)
{
    softSerial_t *self = (softSerial_t *)s;
    if ((s->mode & MODE_TX) == 0) {
        return;
    }

    uint16_t nxt=(s->txBufferHead + 1) % s->txBufferSize;
    if(nxt==s->txBufferTail) {
        // TODO - buffer is full ...  we could wait (if outside of isr), but that could break something important. 
        // only log error end discard character now
        self->transmissionErrors++;
    } else {
        s->txBuffer[s->txBufferHead] = ch;
        s->txBufferHead = nxt;
        // elevate priority to callback level
        uint8_t saved_basepri=__get_BASEPRI();
        __set_BASEPRI(NVIC_BUILD_PRIORITY(0xff, 0xff));  asm volatile ("" ::: "memory");  
        softSerialTryTx(self);
        __set_BASEPRI(saved_basepri);   
    }
}

void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate)
{
    softSerial_t *self = container_of(s, softSerial_t, port);
    // TODO!
    self->port.baudRate = baudRate;
    self->bitTime=(1000000<<8)/baudRate;                     // fractional number of timer ticks per bit, 24.8 fixed point
    self->invBitTime=((long long)baudRate<<16)/1000000;      // scaled inverse bit time. Used to to get bit number (multiplication is faster)
}

void softSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->mode = mode;
}

bool isSoftSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

const struct serialPortVTable softSerialVTable[] = {
    {
        softSerialWriteByte,
        softSerialTotalBytesWaiting,
        softSerialReadByte,
        softSerialSetBaudRate,
        isSoftSerialTransmitBufferEmpty,
        softSerialSetMode,
        softSerialSetDirection,
    }
};

#endif
