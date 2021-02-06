/*
 * packets.h
 *
 *  Created on: 31 янв. 2021 г.
 *      Author: VHEMaster
 */

#ifndef PACKETS_H_
#define PACKETS_H_

#include "main.h"

#define TABLE_SETUPS_MAX 4
#define TABLE_PRESSURES_MAX 24
#define TABLE_ROTATES_MAX 24
#define TABLE_TEMPERATURES_MAX 12
#define TABLE_STRING_MAX 12

typedef enum
{
  ValveAllClosed = 0,
  ValvePetrol = 1,
  ValvePropane = 2,
}eValveChannel;

typedef struct
{
    char name[TABLE_STRING_MAX];

    eValveChannel valve_channel;
    uint32_t valve_timeout;

    float initial_ignition;
    float octane_corrector;

    uint32_t idles_count;
    float idle_ignitions[TABLE_ROTATES_MAX];
    float idle_rotates[TABLE_ROTATES_MAX];

    uint32_t pressures_count;
    float pressures[TABLE_PRESSURES_MAX];

    uint32_t rotates_count;
    float rotates[TABLE_ROTATES_MAX];

    float ignitions[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];

    uint32_t temperatures_count;
    float temperatures[TABLE_TEMPERATURES_MAX];
    float temperature_ignitions[TABLE_TEMPERATURES_MAX];

    float servo_acc[TABLE_TEMPERATURES_MAX];
    float servo_choke[TABLE_TEMPERATURES_MAX];

    uint32_t Reserved[256];
}sAcisIgnTable;

typedef struct
{
    uint8_t isCutoutEnabled;
    uint8_t isTemperatureEnabled;
    uint8_t isEconomEnabled;
    uint8_t isAutostartEnabled;
    uint8_t isIgnitionByHall;
    uint8_t isForceTable;
    uint8_t isHallLearningMode;
    uint8_t isSwitchByExternal;
    uint8_t isEconOutAsStrobe;

    uint8_t switchPos1Table;
    uint8_t switchPos0Table;
    uint8_t switchPos2Table;
    uint8_t forceTableNumber;

    uint8_t Reserved8[256 - 9];

    float InitialRpmThreshold;
    float EconRpmThreshold;
    float CutoutRPM;

    uint32_t Reserved32[256];
}sAcisParams;

typedef struct
{
    uint32_t tables_count;
    sAcisIgnTable tables[TABLE_SETUPS_MAX];
    sAcisParams params;
    uint16_t crc;
}sAcisConfig;



#define PK_GeneralStatusRequestID 1
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;

}PK_GeneralStatusRequest_t;

#define PK_GeneralStatusResponseID 2
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t tablenum;
  uint8_t valvenum;
  uint8_t check;
  char tablename[TABLE_STRING_MAX];
  float RealRPM;
  float RPM;
  float Pressure;
  float Load;
  float IgnitionAngle;
  float IgnitionTime;
  float Voltage;
}PK_GeneralStatusResponse_t;

extern int8_t PK_Copy(void * dest, void * source);

#define PACKET_C(x) extern x##_t x;

PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);

#undef PACKET_C


#endif /* PACKETS_H_ */
