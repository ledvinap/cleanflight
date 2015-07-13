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
#include "flight/pid.h"
#include "flight/altitudehold.h"
#include "io/gps.h"

#include "telemetry/telemetry.h"
#include "telemetry/sport.h"

extern int16_t accSmooth[XYZ_AXIS_COUNT];

void telemetrySPortSerialRxCharCallback(uint16_t data);

timerQueueRec_t telemetrySPortTimerQ;

serialPortMode_t sPortPortConfig = {
    .mode = MODE_RXTX | MODE_SINGLEWIRE | MODE_HALFDUPLEX | MODE_INVERTED | MODE_S_DUALTIMER,
    .baudRate = 57600,
    .rxCallback = telemetrySPortSerialRxCharCallback
};

static serialPort_t *sPortSerialPort = NULL; 
static serialPortConfig_t *portConfig;

static telemetryConfig_t *telemetryConfig;
static bool sPortTelemetryEnabled =  false;
static portSharing_e sPortPortSharing;

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
#define GPS_SPEED_FIRST_ID      0x0830           // in 0.01 km/h ?
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840           // heading;  in hunderds of degree
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850           // hhmmss00 or YYMMDDxx with nonzero xx
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
    tlm_GPS, tlm_GPS_Speed,
#endif
    tlm_Time, tlm_Status
} tlm_Id;

struct tlm_info_s {
    uint8_t id;
    uint16_t delta_t;  // in ms
};

const struct tlm_info_s tlm_info[] = {
    {tlm_Acc, 125},
    {tlm_Vario, 125},
    {tlm_BaroAlt, 250},
    {tlm_Heading, 250},
    {tlm_Temp1, 1000},
    {tlm_Current, 1000},
    {tlm_Voltage, 1000},
    {tlm_Cells, 1000},
    {tlm_Fuel, 1000},
#ifdef GPS
    {tlm_GPS, 500},
    {tlm_GPS_Speed, 500},
#endif
    {tlm_Time, 5000},
    {tlm_Status, 250},  // sent as T2 now
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
        serialWrite(sPortSerialPort, 0x7D);
        v ^= 0x20;
    }
    serialWrite(sPortSerialPort, v);
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
    for(unsigned i = 0; i < 8; i++)
        tx_u8(pkt[i]);
    // start transmitting only after full packet is in queue
    serialUpdateState(sPortSerialPort, ~STATE_RX, STATE_TX | STATE_RX_WHENTXDONE); 
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
    // send also GPS altitude here
    pushPacket(GPS_ALT_FIRST_ID, GPS_altitude * 100);   // send in mm, local in 0.1m
}
#endif

static void tlm_sendStatus(void) {
    uint32_t tmpi;
    // the t1Cnt simply allows the telemetry view to show at least some changes
    static int t1Cnt=0;
    if (++t1Cnt > 2) {
        t1Cnt = 1;
    }
    tmpi = t1Cnt * 10000; // start off with at least one digit so the most significant 0 won't be cut off
    // the Taranis seems to be able to fit 5 digits on the screen
    // the Taranis seems to consider this number a signed 16 bit integer, maximum is 32767, use values up to 20000

    if (ARMING_FLAG(OK_TO_ARM))
        tmpi += 1;
    if (ARMING_FLAG(PREVENT_ARMING))
        tmpi += 2;
    if (ARMING_FLAG(ARMED))
        tmpi += 4;

    if (FLIGHT_MODE(ANGLE_MODE))
        tmpi += 10;
    if (FLIGHT_MODE(HORIZON_MODE))
        tmpi += 20;
    if (FLIGHT_MODE(AUTOTUNE_MODE))
        tmpi += 40;
    if (FLIGHT_MODE(PASSTHRU_MODE))
        tmpi += 40;

    if (FLIGHT_MODE(MAG_MODE))
        tmpi += 100;
    if (FLIGHT_MODE(BARO_MODE))
        tmpi += 200;
    if (FLIGHT_MODE(SONAR_MODE))
        tmpi += 400;

    if (FLIGHT_MODE(GPS_HOLD_MODE))
        tmpi += 1000;
    if (FLIGHT_MODE(GPS_HOME_MODE))
        tmpi += 2000;
    if (FLIGHT_MODE(HEADFREE_MODE))
        tmpi += 4000;

    pushPacket(T2_FIRST_ID, tmpi);
}

// generate packet for given telemetry ID
// return 0 if packet not generated or delay in ms for next call with this id
static int generatePacket(tlm_Id id) {
    switch(id) {
    case tlm_Acc:
        for(int i = 0; i < 3; i++)
            pushPacketS(ACC_FIRST_ID(i), (int)accSmooth[i] * 100 / acc_1G); // send int 0.01G
        break;
    case tlm_Vario:
        pushPacketS(VARIO_FIRST_ID, vario);    // send in 0.01m/s
        break;
    case tlm_BaroAlt:
        pushPacketS(ALT_FIRST_ID, BaroAlt);    // in 0.01m
        break;
    case tlm_Heading:
        pushPacketS(GPS_COURS_FIRST_ID, heading*100);  // internal 1deg, sent in 0.01deg
        break;
    case tlm_Temp1:
        pushPacketS(T1_FIRST_ID, telemTemperature1);  // send in degrees
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
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX))
            tlm_sendGPS();
        else
            return 0;
        break;
    case tlm_GPS_Speed:
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX))
            pushPacket(GPS_SPEED_FIRST_ID, (GPS_speed * 36 + 36 / 2) / 100);  // send in in 0.01km/h, local in 0.1m/s
        break;

#endif
    case tlm_Time: {
        unsigned seconds = millis() / 1000;
        unsigned minutes = seconds / 60;
        unsigned hours = minutes / 60;
        pushPacket(GPS_TIME_DATE_FIRST_ID, ((hours & 0xff) << 24) | ((minutes % 60) << 16) | ((seconds % 60) << 8));
        }
        break;
    case tlm_Status:
        tlm_sendStatus();
        break;
    }
    return tlm_info[id].delta_t; // default replan time
}

void initSPortTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SPORT);
    sPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SPORT);

    // enqueue all packets
    // TODO - handle case when telemetry is not running - there is 32s overflow
    for(unsigned i = 0; i < ARRAYLEN(tlm_info); i++) {
        telemQueueInsert(millis(), i);
    }
    telemPktHead = telemPktTail = 0;
    // timer user to trigger reply after poll
    timerQueue_Config(&telemetrySPortTimerQ, telemetrySPortTimerQCallback);
}

void freeSPortTelemetryPort(void)
{
    closeSerialPort(sPortSerialPort);
    sPortSerialPort = NULL;

    sPortTelemetryEnabled = false;
}

void configureSPortTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    sPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SPORT, &sPortPortConfig);

    if (!sPortSerialPort) {
        return;
    }

    sPortTelemetryEnabled = true;
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

void checkSPortTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(sPortPortSharing);

    if (newTelemetryEnabledValue == sPortTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureSPortTelemetryPort();
    else
        freeSPortTelemetryPort();
}

void handleSPortTelemetry(void)
{
    if (!sPortTelemetryEnabled)
        return;

    if(!telemHeapLen)  // this should never happend
        return;
    while(telemPktQueueEmpty() && tq_cmp(telemHeap[0], millis() << 16) <= 0) { // got packet to send
        tlm_Id id = telemHeap[0] & 0xffff;
        telemQueueDeleteIdx(0);
        int res = generatePacket(id);
        if(res <= 0 || res >= 10000) {   // TODO
            res = 10000; // try 10s later
        }
        telemQueueInsert(millis() + res, id);
    }
}

#endif
