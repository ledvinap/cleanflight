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

/*
 * FrSky SPort Telemetry implementation by silpstream @ rcgroups
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/timer_queue.h"
#include "drivers/serial.h"
#include "io/serial.h"
#include "drivers/pin_debug.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "flight/flight.h"
#include "io/gps.h"

#include "telemetry/telemetry.h"
#include "telemetry/sport.h"

static serialPort_t *sPortPort;

void telemetrySPortSerialRxCharCallback(uint16_t data);

timerQueueRec_t telemetrySPortTimerQ;

serialPortConfig_t sPortPortConfig = {
    .mode = MODE_RXTX | MODE_SINGLEWIRE | MODE_HALFDUPLEX | MODE_INVERTED | MODE_S_DUALTIMER,
    .baudRate = 57600,
    .rxCallback = telemetrySPortSerialRxCharCallback
};

static telemetryConfig_t *telemetryConfig;

extern int16_t telemTemperature1; // FIXME dependency on mw.c

#define CYCLETIME             125

#define DATA_FRAME              0x10

#define ALT_FIRST_ID            0x0100           // baro altitude, in cm
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110           // Z speed, in cm/s
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200           // current, 0.1A
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210           // voltage, in 10mV
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300           // bbbaaaCc - C - cells count, c - cell,
                                                 // aaa - V(c), bbb - V(c+1), 0.002V
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400           // temperature in degrees C ?
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600           // fuel level, in percent
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700           // in 1/100 g
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800           // bit 31: 0=latitude,1=longitude
                                                 // bit 30: sign (1-negative [S,W])
                                                 // bit 0-29: minutes*10000
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840         // heading;  in hunderds of degree
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850         // hhmmss00 or YYMMDDxx with nonzero xx
#define GPS_TIME_DATE_LAST_ID   0x085f
#define A3_FIRST_ID             0x0900
#define A3_LAST_ID              0x090f
#define A4_FIRST_ID             0x0910
#define A4_LAST_ID              0x091f
#define AIR_SPEED_FIRST_ID      0x0a00
#define AIR_SPEED_LAST_ID       0x0a0f
#define RSSI_ID                 0xf101
#define ADC1_ID                 0xf102
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105

// Default sensor data IDs (Physical IDs + CRC)
#define DATA_ID_VARIO            0x00 // 0
#define DATA_ID_FLVSS            0xA1 // 1
#define DATA_ID_FAS              0x22 // 2
#define DATA_ID_GPS              0x83 // 3
#define DATA_ID_RPM              0xE4 // 4
#define DATA_ID_SP2UH            0x45 // 5
#define DATA_ID_SP2UR            0xC6 // 6

#define ACC_FIRST_ID(x)        (ACCX_FIRST_ID + (i) * 0x10)

typedef enum  {
    tlm_Acc = 0, tlm_Vario, tlm_BaroAlt, tlm_Heading, tlm_Temp1, tlm_Current,
    tlm_Voltage, tlm_Cells, tlm_Fuel,
#ifdef GPS
    tlm_GPS,
#endif
    tlm_Time
} tlm_Id;

struct tlm_info_s {
    uint8_t id;
    uint16_t delta_t;  // in ms
};

const struct tlm_info_s tlm_info[] = {
    {tlm_Acc, 125},
    {tlm_Vario, 125},
    {tlm_BaroAlt, 500},
    {tlm_Heading, 500},
    {tlm_Temp1, 1000},
    {tlm_Current, 1000},
    {tlm_Voltage, 1000},
    {tlm_Cells, 1000},
    {tlm_Fuel, 1000},
#ifdef GPS
    {tlm_GPS, 1000},
#endif
    {tlm_Time, 5000},
};


static int telemQueueInsert(uint16_t time, uint16_t data);
static void telemQueueDeleteIdx(unsigned parent);

#define TELEM_PKTQUEUE_LEN 8
static uint8_t telemPktQueue[TELEM_PKTQUEUE_LEN][8];
static unsigned telemPktHead, telemPktTail;

static bool telemPktQueuePush(uint8_t* pkt)
{
    unsigned nxt = (telemPktHead + 1) % TELEM_PKTQUEUE_LEN;
    if(nxt == telemPktTail) return false;
    memcpy(telemPktQueue[telemPktHead], pkt, sizeof(telemPktQueue[telemPktHead]));
    telemPktHead = nxt;
    return true;
}

static void telemPktQueuePop(void)
{
    if(telemPktHead == telemPktTail) return;
    telemPktTail = (telemPktTail + 1) % TELEM_PKTQUEUE_LEN;
}

static uint8_t* telemPktQueueHead(void)
{
    if(telemPktHead == telemPktTail) return NULL;
    return telemPktQueue[telemPktTail];
}

static unsigned telemPktQueueEmpty(void)
{
    return telemPktHead == telemPktTail;
}

void set_crc(uint8_t* pkt)
{
    unsigned crc = 0;
    for(unsigned i = 0; i < 7; i++)
        crc += pkt[i];
    crc = (crc & 0xff) + (crc >> 8);
    crc = (crc & 0xff) + (crc >> 8);
    pkt[7] = ~crc;
}

void tx_u8(uint8_t v)
{
    if(v == 0x7d || v == 0x7e) {
        serialWrite(sPortPort, 0x7D);
        v ^= 0x20;
    }
    serialWrite(sPortPort, v);
}


void telemetrySPortSerialRxCharCallback(uint16_t data)
{
    // TODO !!
    static uint16_t rcvd;
    rcvd <<= 8;
    rcvd |= data&0xff;
    if(rcvd == ((0x7e << 8) | DATA_ID_VARIO)
       && telemPktQueueHead() != NULL) {
        pinDbgHi(DBP_TELEMETRY_SPORT_REPLYWAIT);
        timerQueue_Start(&telemetrySPortTimerQ, 100);
    }
}

void telemetrySPortTimerQCallback(timerQueueRec_t *cb)
{
    UNUSED(cb);
    pinDbgLo(DBP_TELEMETRY_SPORT_REPLYWAIT);
    uint8_t *pkt = telemPktQueueHead();
    if(!pkt) return;
    serialSetDirection(sPortPort, STATE_TX);
    for(unsigned i = 0; i < 8; i++)
        tx_u8(pkt[i]);
    serialUpdateState(sPortPort, ~0, STATE_RX_WHENTXDONE);
    telemPktQueuePop();
}

static void pushPacket(uint16_t id, uint32_t value)
{
    uint8_t pkt[8];
    pkt[0] = 0x10;
    memcpy(pkt + 1, &id, 2);
    memcpy(pkt + 3, &value, 4);
    set_crc(pkt);
    telemPktQueuePush(pkt);
}

static void pushPacketS(uint16_t id, int32_t value)
{
    pushPacket(id, (uint32_t)value);
}

static void tlm_sendCells(void)
{
    static uint8_t currentCellIdx = 0;
    if(currentCellIdx >= batteryCellCount)  // do it first in case batteryCellCount was decreased
        currentCellIdx = 0;
    uint32_t val = 0;
    val |= (batteryCellCount << 4) | currentCellIdx;
    uint16_t cellVoltage = ((int)vbat * 50 / batteryCellCount) & 0xfff;  // scale to 10mV
    val |= cellVoltage << 8;                // lower cell voltage
    if(currentCellIdx + 1 < batteryCellCount)
        val |= cellVoltage << 20;           // upper cell voltage if cell exists
    pushPacket(CELLS_FIRST_ID, val);
    currentCellIdx += 2;                    // 2 cell voltages sent
}

#ifdef GPS
void tlm_sendGPS(void)
{
    // GPS_COORS is in 1e-7 degrees
    uint32_t val;
    val = abs(GPS_coord[LAT]) * 3 / 50;  // *60/1000
    if(GPS_coord[LAT]<0) val |= 1 << 30;
    pushPacket(GPS_LONG_LATI_FIRST_ID, val);
    val = (abs(GPS_coord[LON]) * 3 / 50) | (1 << 31);
    if(GPS_coord[LAT] < 0) val |= 1 << 30;
    pushPacket(GPS_LONG_LATI_FIRST_ID, val);
}
#endif

// generate packet for given telemetry ID
// return 0 if packet not generated or delay in ms for next call with this id
static int generatePacket(tlm_Id id) {
    switch(id) {
    case tlm_Acc:
        for(int i = 0; i < 3; i++)
            pushPacketS(ACC_FIRST_ID(i), (int)accSmooth[i] * 100 / acc_1G); // convert to 8.8 fixed point
        break;
    case tlm_Vario:
        pushPacketS(VARIO_FIRST_ID, vario);
        break;
    case tlm_BaroAlt:
        pushPacketS(ALT_FIRST_ID, BaroAlt);
        break;
    case tlm_Heading:
        pushPacketS(GPS_COURS_FIRST_ID, heading);  // TODO - scaling is probably necessary
        break;
    case tlm_Temp1:
        pushPacketS(T1_FIRST_ID, telemTemperature1);  // TODO - maybe /10 ?
        break;
    case tlm_Current:
        if (feature(FEATURE_VBAT))
            pushPacketS(CURR_FIRST_ID, (amperage + 5) / 10);
        else
            return 0;
        break;
    case tlm_Voltage:
        if (feature(FEATURE_VBAT))
            pushPacket(VFAS_FIRST_ID, vbat * 10);
        else
            return 0;
        break;
    case tlm_Cells:
        if (feature(FEATURE_VBAT))
            tlm_sendCells();
        else
            return 0;
        break;
    case tlm_Fuel:
        if (feature(FEATURE_VBAT))
            pushPacket(FUEL_FIRST_ID, mAhDrawn); // TODO - value should be in percent
        else
            return 0;
        break;
#ifdef GPS
    case tlm_GPS:
        if (sensors(SENSOR_GPS))
            tlm_sendGPS();
        else
            return 0;
        break;
#endif
    case tlm_Time: {
        unsigned seconds = millis() / 1000;
        unsigned minutes = seconds / 60;
        unsigned hours = minutes / 60;
        pushPacket(GPS_TIME_DATE_FIRST_ID, ((hours & 0xff) << 24) | ((minutes % 60) << 16) | ((seconds % 60) << 8));
        }
        break;
    }
    return tlm_info[id].delta_t; // default replan time
}

void initSPortTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    // enqueue all packets
    // TODO - handle case when telemetry is not running - there is 32s overflow
    for(unsigned i = 0; i < ARRAYLEN(tlm_info); i++) {
        telemQueueInsert(millis(), i);
    }
    telemPktHead = telemPktTail = 0;
    // timer user to trigger reply after poll
    timerQueue_Config(&telemetrySPortTimerQ, telemetrySPortTimerQCallback);
}

static serialPortConfig_t previousSerialConfig = SERIAL_CONFIG_INIT_EMPTY;

void freeSPortTelemetryPort(void)
{
    serialRelease(sPortPort);
    serialConfigure(sPortPort, &previousSerialConfig);
    endSerialPortFunction(sPortPort, FUNCTION_TELEMETRY);
}

void configureSPortTelemetryPort(void)
{
    sPortPort = findOpenSerialPort(FUNCTION_TELEMETRY);
    if (sPortPort) {
        serialGetConfig(sPortPort, &previousSerialConfig);
        serialRelease(sPortPort);
        //waitForSerialPortToFinishTransmitting(sPortPort); // FIXME locks up the system
        serialConfigure(sPortPort, &sPortPortConfig);
        beginSerialPortFunction(sPortPort, FUNCTION_TELEMETRY);
    } else {
        sPortPort = openSerialPort(FUNCTION_TELEMETRY, &sPortPortConfig);
    }
}

#define TELEM_HEAP_LEN ARRAYLEN(tlm_info)
uint32_t telemHeap[TELEM_HEAP_LEN];
unsigned telemHeapLen = 0;

static inline int16_t tq_cmp(uint32_t a, uint32_t b)
{
    return (a - b) >> 16;
}

// insert new timer into queue
// return position where new record was inserted
static int telemQueueInsert(uint16_t time, uint16_t data)
{
    uint32_t rec = (time << 16) | data;
    unsigned parent, child;
    child = telemHeapLen++;
    while(child) {
        parent = (child - 1) / 2;
        if(tq_cmp(telemHeap[parent], rec) <= 0) break;
        telemHeap[child] = telemHeap[parent];
        child = parent;
    }
    telemHeap[child] = rec;
    return child;
}

// remove element at given index from queue
static void telemQueueDeleteIdx(unsigned parent)
{
    if(telemHeapLen == 0) return;
    unsigned child;
    uint32_t last = telemHeap[--telemHeapLen];
    while ((child = (2 * parent) + 1) < telemHeapLen) {
        if (child + 1 < telemHeapLen
            && tq_cmp(telemHeap[child], telemHeap[child + 1]) >= 0)
            ++child;
        if(tq_cmp(last, telemHeap[child]) <= 0)
            break;
        telemHeap[parent] = telemHeap[child];
        parent = child;
    }
    telemHeap[parent] = last;
}

void handleSPortTelemetry(void)
{
    if(!telemHeapLen)  // this should never happend
        return;
    while(telemPktQueueEmpty()) {
        if(tq_cmp(telemHeap[0], millis() << 16) <= 0) { // time is up
            tlm_Id id = telemHeap[0] & 0xffff;
            telemQueueDeleteIdx(0);
            int res = generatePacket(id);
            if(res <= 0 || res >= 10000) {   // TODO
                res = 10000; // try 10s later
            }
            telemQueueInsert(millis() + res, id);
        }
    }
}

uint32_t getSPortTelemetryProviderBaudRate(void) {
    return sPortPortConfig.baudRate;
}
#endif
