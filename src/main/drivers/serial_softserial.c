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

#include "nvic.h"
#include "system.h"
#include "gpio.h"
#include "timer.h"
#include "callback.h"

#include "pwm_mapping.h"

#include "serial.h"
#include "serial_softserial.h"

#if defined(STM32F10X_MD) || defined(CHEBUZZF3)
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 11 // PWM 5
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 12 // PWM 6
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 12 // PWM 7
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 13 // PWM 8
#endif

#if defined(STM32F303) && !defined(CHEBUZZF3)
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
// TODO debug
    gpio_config_t cfg;
    
    cfg.pin = Pin_0|Pin_8;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_10MHz;
    gpioInit(GPIOA, &cfg);

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
        timerQueue_Config(&self->rxTimerQ, softSerialRxTimeoutEvent);
        timerOut_Config(&self->txTimerCh, 
                        self->txTimerHardware, TYPE_SOFTSERIAL_RXTX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY), 
                        &self->txCallback, (inversion?TIMEROUT_INVERTED:0)|TIMEROUT_WAKEONEMPTY);
        timerOut_Release(&self->txTimerCh);
        timerIn_Config(&self->rxTimerCh, 
                       self->rxTimerHardware, TYPE_SOFTSERIAL_RXTX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY), 
                       &self->rxCallback, &self->rxTimerQ, 
                       (inversion?0:TIMERIN_RISING)|TIMERIN_POLARITY_TOGGLE|(inversion?TIMERIN_IPD:0));
        timerIn_SetBuffering(&self->rxTimerCh, 0);
    } else {
        if(mode & MODE_TX) {
            timerOut_Config(&self->txTimerCh, 
                            self->txTimerHardware, TYPE_SOFTSERIAL_TX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY),
                            &self->txCallback, (inversion?0:TIMEROUT_INVERTED)|TIMEROUT_WAKEONEMPTY|TIMEROUT_WAKEONLOW);
            callbackRegister(&self->txCallback, softSerialTxCallback);
            delay(1); // TODO - only for testing
        }
        if(mode & MODE_RX) {
            timerQueue_Config(&self->rxTimerQ, softSerialRxTimeoutEvent);
            callbackRegister(&self->rxCallback, softSerialRxCallback);
            timerIn_Config(&self->rxTimerCh, 
                           self->rxTimerHardware,  TYPE_SOFTSERIAL_RX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY), 
                           &self->rxCallback, &self->rxTimerQ, 
                           (inversion?TIMERIN_RISING:0)|TIMERIN_POLARITY_TOGGLE|(inversion?TIMERIN_IPD:0)|TIMERIN_QUEUE_BUFFER);
            self->rxTimerCh.timeout=((self->bitTime*RX_TOTAL_BITS)>>8)+50; // 50 us after stopbit
            callbackTrigger(&self->rxCallback);                           // setup timeouts correctly
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
    
        // we need to enque odxd number of intervals
        // there is toggle on start of each interval, first interval starts in idle state
        unsigned bitPos=0;
        uint16_t lastEdgePos=0;
        while(byteToSend) {
            unsigned bitcnt=(byteToSend&1)?__builtin_ctz(~byteToSend):__builtin_ctz(byteToSend);
            byteToSend>>=bitcnt;
            bitPos+=bitcnt;
            uint16_t edgePos=(bitPos*self->bitTime)>>8;
            timerOut_QPush(&self->txTimerCh, edgePos-lastEdgePos, 0);
            lastEdgePos=edgePos;
        }
        timerOut_QCommit(&self->txTimerCh);   // this will start transmission if necessary
    }
}
#define STARTBIT_MASK 0x0001
#define STOPBIT_MASK ((0xffff<<(RX_BITS+1))&0xffff)   // check event 'virtual' stopbits, useful to signal errors

void srStoreByte(softSerial_t *self, uint16_t shiftRegister) {
    if((shiftRegister & (STARTBIT_MASK|STOPBIT_MASK)) != (0|STOPBIT_MASK)) {
        self->receiveErrors++;
        digitalToggle(GPIOA, Pin_8);
        return;
    }

    // TODO - check parity if in sbus mode
    uint8_t byte=shiftRegister>>1; // shift startbit out

    if (self->port.callback) {
        self->port.callback(byte);
    } else {
        self->port.rxBuffer[self->port.rxBufferHead] = byte;
        self->port.rxBufferHead = (self->port.rxBufferHead + 1) % self->port.rxBufferSize;
    }
}

// TODO
static inline int16_t cmp16(uint16_t a, uint16_t b) 
{
    return a-b;
}


void softSerialRxProcess(softSerial_t *self)
{
    uint16_t capture0, capture1, symbolStart, symbolEnd;
    do { // return here if late edge was caught
        // always process whole symbol; first captured edge must be startbit
        while(timerIn_QPeek2(&self->rxTimerCh, &symbolStart, &capture1)) {
            symbolEnd=symbolStart+((self->bitTime*(2*RX_TOTAL_BITS-1)/2)>>8);   // half bit shorter
            if(cmp16(timerIn_getTimCNT(&self->rxTimerCh), symbolEnd)<0) {
                timerQueue_Start(&self->rxTimerQ, symbolEnd-timerIn_getTimCNT(&self->rxTimerCh)+20);  // add some time to wait for next startbit
                return;   // symbol is not finished yet
            }
            symbolStart-=((self->bitTime / 2) >> 8); // offset startbit edge to get correct rounding
            uint16_t rxShiftRegister=0xffff;
            int bitIdxLast=-1;    
            int bitIdx0=0;        // edge from 1 to 0 (and position of startbit)
            int bitIdx1;          // edge from 0 to 1
            // jump inside loop - startbit is always at known position
            timerIn_QPop2(&self->rxTimerCh);
            goto edge1;
            while(timerIn_QPeek2(&self->rxTimerCh, &capture0, &capture1)) {
                if(cmp16(capture0, symbolEnd)>0) {
                    // this edge is in next symbol, process received data
                    break;
                }
                timerIn_QPop2(&self->rxTimerCh);
                bitIdx0=((unsigned long)((capture0-symbolStart)&0xffff)*self->invBitTime)>>16;
            edge1:
                bitIdx1=((unsigned long)((capture1-symbolStart)&0xffff)*self->invBitTime)>>16;

                if(bitIdxLast>=bitIdx0 || bitIdx0>=bitIdx1) {
                    // invalid bit interval or repeated edge in same bit
                    rxShiftRegister&=~0x8000;  // non-transmitted bits are used as error flags, TODO - symbolic constants
                    break;
                }
                bitIdxLast=bitIdx1;
                rxShiftRegister&=~(0xffff<<bitIdx0);   // clear all bits >= bitIdx0
                rxShiftRegister|=0xffff<<bitIdx1;      // set all bits >= bitIdx1
            }
            srStoreByte(self, rxShiftRegister);
        }
        // no data in queue, setup edge timeout
    } while(!timerIn_ArmEdgeTimeout(&self->rxTimerCh));   // maybe goto will be more readable here ... 
}

void softSerialRxCallback(callbackRec_t *cb)
{
    softSerial_t *self=container_of(cb, softSerial_t, rxCallback);
    softSerialRxProcess(self); 
}

void softSerialRxTimeoutEvent(timerQueueRec_t *tq_ref)
{
    softSerial_t* self=container_of(tq_ref, softSerial_t, rxTimerQ);
    softSerialRxProcess(self);
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
