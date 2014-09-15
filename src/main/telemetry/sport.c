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

#include "platform.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "io/serial.h"

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
#define SPORT_BAUDRATE 57600
#define SPORT_INITIAL_PORT_MODE MODE_RX|MODE_TX|MODE_SINGLEWIRE

static telemetryConfig_t *telemetryConfig;

extern int16_t telemTemperature1; // FIXME dependency on mw.c

#define CYCLETIME             125

#define DATA_FRAME              0x10

#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
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

#define ACC_FIRST_ID(x)        (ACCX_FIRST_ID+(i)*0x10)

uint16_t pkt_id;
uint32_t pkt_value;
uint16_t pkt_crc;

void tx_crc_start(void)
{
    pkt_crc=0;
}

void tx_u8(uint8_t v)
{
    pkt_crc+=v;
    serialWrite(sPortPort, v);
}

void tx_u16(uint16_t v)
{
    tx_u8((v&0x00ff)>>0);
    tx_u8((v&0xff00)>>8);
}

void tx_u32(uint16_t v)
{
    tx_u8((v&0x000000ff)>>0);
    tx_u8((v&0x0000ff00)>>8);
    tx_u8((v&0x00ff0000)>>16);
    tx_u8((v&0xff000000)>24);
}

void tx_crc(void)
{
    while(pkt_crc&0xff00)
        pkt_crc=(pkt_crc&0xff)+(pkt_crc>>8);
    serialWrite(sPortPort,pkt_crc);
}

void telemetrySPortSerialRxCharCallback(uint16_t data)
{
    // TODO !!
    static uint16_t rcvd;
    rcvd<<=8;
    rcvd|=data&0xff;
    if(rcvd==((0x7e<<8)|DATA_ID_GPS) && pkt_id!=0) {
        serialSetDirection(sPortPort, DIRECTION_TX);
        tx_crc_start();
        tx_u8(DATA_FRAME);
        tx_u16(pkt_id);
        tx_u32(pkt_value);
        tx_crc();
        serialSetDirection(sPortPort, DIRECTION_RX_WHENTXDONE);
    }
}

void telemetrySPortSerialTxDoneCallback(void)
{
    serialSetDirection(sPortPort, false);
}


static void pushPacket(uint16_t id, uint32_t value)
{
    pkt_id=id;
    pkt_value=value;
}

static void generatePacket(void) {
    static int idx=0;
    switch(idx) {
    case 0:
        for(int i=0;i<3;i++) 
            pushPacket(ACC_FIRST_ID(i), ((float)accSmooth[i] / acc_1G) * 1000);
    case 1:
        pushPacket(VARIO_FIRST_ID, vario);
        break;
    case 2:
        pushPacket(ALT_FIRST_ID, BaroAlt);
        break;
    case 3:
        pushPacket(GPS_COURS_FIRST_ID, heading);  // TODO - scaling is probably necessary
        break;
    case 4:
        pushPacket(T1_FIRST_ID, telemTemperature1);  // TODO - maybe /10 ?
        break;
    }
#if 0
    case 5:
        if (feature(FEATURE_VBAT)) {
            sendVoltage();
            break;
        }
    case 6:
        if (feature(FEATURE_VBAT)) {
            sendVoltageAmp();
            break;
        }
    case 7:
        if (feature(FEATURE_VBAT)) {
            sendAmperage();
            break;
        }
    case 8:
        if (feature(FEATURE_VBAT)) {
            sendFuelLevel();
        }
    case 9:
#ifdef GPS
        if (sensors(SENSOR_GPS))
            sendGPS();
#endif
    case 10:
        sendTime();
    }
#endif
}

void initSPortTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
}

static portMode_t previousPortMode;
static uint32_t previousBaudRate;

void freeSPortTelemetryPort(void)
{
    // FIXME only need to reset the port if the port is shared
    serialSetMode(sPortPort, previousPortMode);
    serialSetBaudRate(sPortPort, previousBaudRate);

    endSerialPortFunction(sPortPort, FUNCTION_TELEMETRY);
}

void configureSPortTelemetryPort(void)
{
    sPortPort = findOpenSerialPort(FUNCTION_TELEMETRY);
    if (sPortPort) {
        previousPortMode = sPortPort->mode;
        previousBaudRate = sPortPort->baudRate;

        //waitForSerialPortToFinishTransmitting(sPortPort); // FIXME locks up the system

        serialSetBaudRate(sPortPort, SPORT_BAUDRATE);
        serialSetMode(sPortPort, SPORT_INITIAL_PORT_MODE);
        beginSerialPortFunction(sPortPort, FUNCTION_TELEMETRY);
    } else {
        sPortPort = openSerialPort(FUNCTION_TELEMETRY, telemetrySPortSerialRxCharCallback, SPORT_BAUDRATE, SPORT_INITIAL_PORT_MODE, telemetryConfig->frsky_inversion);

        // FIXME only need these values to reset the port if the port is shared
        previousPortMode = sPortPort->mode;
        previousBaudRate = sPortPort->baudRate;
    }
}


void handleSPortTelemetry(void)
{

    if(pkt_id) 
        return;

    generatePacket(); 
}

uint32_t getSPortTelemetryProviderBaudRate(void) {
    return SPORT_BAUDRATE;
}
#endif
