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

#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)

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
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 10 // PWM 6
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 12 // PWM 7
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 13 // PWM 8
#endif

#if defined(STM32F303) && !defined(CHEBUZZF3)
#define SOFT_SERIAL_1_TIMER_RX_HARDWARE 8 // PWM 9
#define SOFT_SERIAL_1_TIMER_TX_HARDWARE 9 // PWM 10
#define SOFT_SERIAL_2_TIMER_RX_HARDWARE 10 // PWM 11
#define SOFT_SERIAL_2_TIMER_TX_HARDWARE 11 // PWM 12
#endif

#define SYM_DATA_BITS        8
#define SYM_TOTAL_BITS       (1+SYM_DATA_BITS+1)          // start/data/stop
#define SYM_TOTAL_BITS_SBUS  (1+SYM_DATA_BITS+1+2)        // start/data/parity/stop

#if defined(USE_SOFTSERIAL1) && defined(USE_SOFTSERIAL2)
#define MAX_SOFTSERIAL_PORTS 2
#else
#define MAX_SOFTSERIAL_PORTS 1
#endif

extern const struct serialPortVTable softSerialVTable[];
softSerial_t softSerialPorts[MAX_SOFTSERIAL_PORTS];

void softSerialTxCallback(callbackRec_t *cb);
void softSerialRxCallback(callbackRec_t *cb);
void softSerialTryTx(softSerial_t* self);
void softSerialRxTimeoutEvent(timerQueueRec_t *tq_ref);
void softSerialConfigure(serialPort_t *serial, const serialPortConfig_t *config);
void softSerialSetState(serialPort_t *serial, portState_t newState);

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

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, const serialPortConfig_t *config)
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
    softSerialConfigure(&self->port, config);
    return &self->port;
}

void softSerialConfigure(serialPort_t *serial, const serialPortConfig_t *config)
{
    softSerial_t *self=container_of(serial, softSerial_t, port);
    
    uint32_t baud=config->baudRate;
    portMode_t mode=config->mode;
    portState_t state=0;

    if(config->mode==0)   // prevent reconfiguration with empty config
        return;

    // fix mode if caller got it wrong
    if((mode & MODE_RXTX) == MODE_RXTX && self->txTimerHardware == self->rxTimerHardware)
        mode |= MODE_SINGLEWIRE;
    if(mode & MODE_SINGLEWIRE)
        mode |= MODE_HALFDUPLEX;
    if(mode & MODE_HALFDUPLEX) {
        mode |= MODE_RXTX;
        if(self->txTimerHardware != self->rxTimerHardware)
            mode |= MODE_DUALTIMER;     // we have two channels, so use them 
    }
    
    self->port.baudRate = baud;
    self->port.mode = mode;
    self->port.rxCallback = config->rxCallback;

    resetBuffers(self);

    self->transmissionErrors = 0;
    self->receiveErrors = 0;
    
    self->bitTime=(1000000<<8)/baud;                             // fractional number of timer ticks per bit, 24.8 fixed point
    self->invBitTime=((long long)baud<<16)/1000000;              // scaled inverse bit time. Used to to get bit number (multiplication is faster)
    int symbolBits = mode & MODE_SBUS ? SYM_TOTAL_BITS_SBUS : SYM_TOTAL_BITS; 
    self->symbolLength = (self->bitTime * (2 * symbolBits - 1) / 2) >> 8;  //  symbol ends in middle of last stopbit
    

    if(mode & MODE_SINGLEWIRE) {
        // in sinlgewire we start in RX mode
        // also use RX pin only
        self->txTimerHardware=self->rxTimerHardware;
        self->directionTx=false;
    }
    if(mode & MODE_TX) {
        callbackRegister(&self->txCallback, softSerialTxCallback);
        timerOut_Config(&self->txTimerCh, 
                        self->txTimerHardware, TYPE_SOFTSERIAL_TX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY),
                        &self->txCallback, (mode&MODE_INVERTED?0:TIMEROUT_START_HI)|TIMEROUT_WAKEONEMPTY|TIMEROUT_WAKEONLOW);
        state|=STATE_TX;
    }
    if(mode & MODE_HALFDUPLEX) {  
        // release tx channel in halfduplex mode before configuring RX (timer may be shared)
        // TODO - correctly handle pin in halfduplex on two pins
        timerOut_Release(&self->txTimerCh);
        state &= ~STATE_TX;
    }
    if(mode & MODE_RX) {
        // TODO - in dualtimer case whe should chech that second channel is available and fallback to single-channel mode in neccesary
        // Or fail to open port - could be safer when DMA is used (much higher processor load can be dangerous)
        callbackRegister(&self->rxCallback, softSerialRxCallback);
        timerQueue_Config(&self->rxTimerQ, softSerialRxTimeoutEvent);
        timerIn_Config(&self->rxTimerCh, 
                       self->rxTimerHardware,  (mode & MODE_SINGLEWIRE) ? TYPE_SOFTSERIAL_RXTX : TYPE_SOFTSERIAL_RX, NVIC_BUILD_PRIORITY(TIMER_IRQ_PRIORITY, TIMER_IRQ_SUBPRIORITY), 
                       &self->rxCallback, &self->rxTimerQ, 
                       ((mode & MODE_INVERTED) ? TIMERIN_RISING : 0) 
                       | ((mode & MODE_DUALTIMER) ? TIMERIN_QUEUE_DUALTIMER : TIMERIN_POLARITY_TOGGLE) 
                       | ((mode & MODE_INVERTED) ? TIMERIN_IPD : 0) 
                       | TIMERIN_QUEUE_BUFFER);
        self->rxTimerCh.timeout = self->symbolLength + 50; // 50 us after stopbit (make it configurable)
        state|=STATE_RX;
        callbackTrigger(&self->rxCallback);                                  // setup timeouts correctly
    } 
    self->port.state=state;
}

void softSerialRelease(serialPort_t *serial)
{
    softSerial_t *self=container_of(serial, softSerial_t, port);
   
    portMode_t mode=self->port.mode;

    softSerialSetState(&self->port, 0);  // disable RX and TX first
    self->port.state=0;
    
    if(mode & MODE_TX) {
        callbackRelease(&self->txCallback);
    }
    if(mode & MODE_RX) {
        callbackRelease(&self->rxCallback);
        timerQueue_Release(&self->rxTimerQ);
    }
    self->port.mode=0;
}

void softSerialGetConfig(serialPort_t *serial, serialPortConfig_t* config)
{
    softSerial_t* self=container_of(serial, softSerial_t, port);
    config->baudRate=(1000000 <<8 ) / self->bitTime;
    config->mode=self->port.mode;
    config->rxCallback = self->port.rxCallback;
}

static void resetBuffers(softSerial_t *softSerial)
{
    softSerial->port.rxBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.rxBuffer = softSerial->rxBuffer;
    softSerial->port.rxBufferTail = 0;
    softSerial->port.rxBufferHead = 0;

    softSerial->port.txBuffer = softSerial->txBuffer;
    softSerial->port.txBufferSize = SOFTSERIAL_BUFFER_SIZE;
    softSerial->port.txBufferTail = 0;
    softSerial->port.txBufferHead = 0;
}

// this interface needs to be changes - it is too complicated now 
void softSerialSetState(serialPort_t *serial, portState_t newState)
{
    softSerial_t* self=container_of(serial, softSerial_t, port);
   
    // elevate priority to CALLBACK to prevent race with serial handlers
    uint8_t saved_basepri = __get_BASEPRI();
    __set_BASEPRI(NVIC_BUILD_PRIORITY(CALLBACK_IRQ_PRIORITY, CALLBACK_IRQ_SUBPRIORITY)); asm volatile ("" ::: "memory");
    if(newState & STATE_CMD_SET) {
        newState = self->port.state | newState;
    } else if(newState & STATE_CMD_CLEAR) {
        newState = self->port.state & ~newState;
    }
#endif

    if(newState & STATE_RX_WHENTXDONE) {
        // first check if there is something in buffer. 
        if(isSoftSerialTransmitBufferEmpty(&self->port) && (timerOut_QLen(&self->txTimerCh) == 0)) {
            // Return to RX immediately 
            newState &= ~STATE_TX;
            newState |= STATE_RX;
        } else {
            self->directionRxOnDone = true;
            digitalHi(GPIOA, Pin_8);
        }
    }
        
    // release channels first
    if(!(newState & STATE_TX) && (self->port.state & STATE_TX)) {
        timerOut_Release(&self->txTimerCh);
        self->port.state &= ~STATE_TX;
        self->directionTx = false;
    }
    if(!(newState & STATE_RX) && (self->port.state & STATE_RX)) {
        timerIn_Release(&self->rxTimerCh);
        self->port.state &= ~STATE_RX;
    }


    if((newState & STATE_RX) && !(self->port.state & STATE_RX)) {
        if(self->port.mode & MODE_HALFDUPLEX) 
            self->directionTx=false;
        timerIn_Restart(&self->rxTimerCh);
        self->directionRxOnDone = false;
        digitalLo(GPIOA, Pin_8);
        self->port.state |= STATE_RX;
    }
     
    if((newState & STATE_TX) && !(self->port.state & STATE_TX)) {
        if(self->port.mode & MODE_HALFDUPLEX) {
            self->directionTx=true;       

        }
        timerOut_Restart(&self->txTimerCh);
        self->port.state |= STATE_TX;
    }
    __set_BASEPRI(saved_basepri);
}

void softSerialTxCallback(callbackRec_t *cb)
{
    softSerial_t *self=container_of(cb, softSerial_t, txCallback);;
    digitalToggle(GPIOA, Pin_0);
    softSerialTryTx(self);

    if(self->directionRxOnDone && timerOut_QLen(&self->txTimerCh)==0) {
        digitalToggle(GPIOA, Pin_0);
        softSerialSetState(&self->port, STATE_RX);
    }
}

void softSerialTryTx(softSerial_t* self) {
    while(!isSoftSerialTransmitBufferEmpty(&self->port)           // do we have something to send?  
          && timerOut_QSpace(&self->txTimerCh) > ((self->port.mode & MODE_SBUS) ? SYM_TOTAL_BITS_SBUS : SYM_TOTAL_BITS)) {  // we need space for whole byte 

    
        uint16_t byteToSend = self->port.txBuffer[self->port.txBufferTail];
        self->port.txBufferTail=(self->port.txBufferTail+1)%self->port.txBufferSize; 
    
        if(self->port.mode & MODE_SBUS) {
            byteToSend |= (__builtin_parity(byteToSend))<<SYM_DATA_BITS;      // parity bit
            byteToSend |= 0x03 << (SYM_DATA_BITS+1);                           // 2x stopbit
            byteToSend<<=1;                                                    // startbit      
        } else {
            byteToSend|=1<<SYM_DATA_BITS;                                      // stopbit
            byteToSend<<=1;                                                    // startbit
        }
        
        // we need to enque odd number of intervals
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
#define STOPBIT_MASK ((0xffff<<(SYM_DATA_BITS+1))&0xffff)   // check event 'virtual' stopbits, useful to signal errors

#define STOPBIT_MASK_SBUS ((0xffff<<(SYM_DATA_BITS+1+1))&0xffff)
#define PARITY_MASK_SBUS ((~(STOPBIT_MASK_SBUS|STARTBIT_MASK))&0xffff)

void softSerialStoreByte(softSerial_t *self, uint16_t shiftRegister) {
    if(self->port.mode & MODE_SBUS) {
        if((shiftRegister & (STARTBIT_MASK|STOPBIT_MASK_SBUS)) != (0|STOPBIT_MASK_SBUS)
           || __builtin_parity(shiftRegister & PARITY_MASK_SBUS) != 0) {
            self->receiveErrors++;
            return;
        }
    } else {
        if((shiftRegister & (STARTBIT_MASK|STOPBIT_MASK)) != (0|STOPBIT_MASK)) {
            self->receiveErrors++;
            return;
        }
    }
    uint8_t byte=shiftRegister>>1; // shift startbit out

    if (self->port.rxCallback) {
        self->port.rxCallback(byte);
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
            symbolEnd=symbolStart+self->symbolLength;   // half bit shorter
            if(timerIn_QLen(&self->rxTimerCh)<SYM_TOTAL_BITS_SBUS                // process data if enough bits for symbol was received to prevent problems with timerCnt overflow
               && cmp16(timerIn_getTimCNT(&self->rxTimerCh), symbolEnd)<0) {
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
            softSerialStoreByte(self, rxShiftRegister);
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

    return (s->port.rxBufferHead - s->port.rxBufferTail) & (s->port.rxBufferSize - 1);
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

bool isSoftSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->txBufferHead == instance->txBufferTail;
}

void  softSerialConfigurationHandler(serialPort_t *serial, portConfigOperation_t op, serialPortConfig_t* config)
{
    switch(op) {
    case OP_CONFIGURE:
        softSerialConfigure(serial, config);
        break;
    case OP_GET_CONFIG:
        softSerialGetConfig(serial, config);
        break;
    case OP_RELEASE:
        softSerialRelease(serial);
        break;
    }    
}

const struct serialPortVTable softSerialVTable[] = {
    {
        softSerialWriteByte,
        softSerialTotalBytesWaiting,
        softSerialReadByte,
        isSoftSerialTransmitBufferEmpty,
        softSerialSetState,
        softSerialConfigurationHandler
    }
};

#endif
