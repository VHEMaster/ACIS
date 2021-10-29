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

#define PACKET_TABLE_MAX_SIZE 224
#define PACKET_CONFIG_MAX_SIZE PACKET_TABLE_MAX_SIZE

#define DRAG_MAX_POINTS 3072
#define DRAG_POINTS_DISTANCE 20000

typedef struct
{
    float RPM;
    float Pressure;
    float Load;
    float Ignition;
    uint32_t Time;
}sDragPoint;

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
    int32_t valve_timeout;

    float initial_ignition;
    float octane_corrector;

    int32_t idles_count;
    float idle_ignitions[TABLE_ROTATES_MAX];
    float idle_rotates[TABLE_ROTATES_MAX];

    int32_t pressures_count;
    float pressures[TABLE_PRESSURES_MAX];

    int32_t rotates_count;
    float rotates[TABLE_ROTATES_MAX];

    float ignitions[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];

    int32_t temperatures_count;
    float temperatures[TABLE_TEMPERATURES_MAX];
    float temperature_ignitions[TABLE_TEMPERATURES_MAX];

    float servo_acc[TABLE_TEMPERATURES_MAX];
    float servo_choke[TABLE_TEMPERATURES_MAX];

    float fuel_rate;
    float fuel_volume;

    int32_t Reserved[62];
}sAcisIgnTable __attribute__((aligned(32)));

typedef struct
{
    int32_t isCutoffEnabled;
    int32_t isTemperatureEnabled;
    int32_t isEconomEnabled;
    int32_t isAutostartEnabled;
    int32_t isIgnitionByHall;
    int32_t isHallLearningMode;
    int32_t isSwitchByExternal;
    int32_t isEconOutAsStrobe;
    int32_t isForceTable;
    int32_t forceTableNumber;
    int32_t switchPos1Table;
    int32_t switchPos0Table;
    int32_t switchPos2Table;

    float EconRpmThreshold;
    float CutoffRPM;
    int32_t CutoffMode;
    float CutoffAngle;
    int32_t isEconIgnitionOff;
    int32_t isForceIdle;
    int32_t engineVolume;
    int32_t isForceIgnition;
    int32_t forceIgnitionAngle;
    int32_t hallLearningTable;

    int32_t Reserved32[24];
}sAcisParams;

typedef struct
{
    int32_t tables_count;
    sAcisParams params;
    sAcisIgnTable tables[TABLE_SETUPS_MAX];
    uint32_t version;
    uint16_t crc;
}sAcisConfig;



#define PK_PingID 1
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t RandomPing;

}PK_Ping_t;

#define PK_PongID 2
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t RandomPong;

}PK_Pong_t;

#define PK_GeneralStatusRequestID 3
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;

}PK_GeneralStatusRequest_t;

#define PK_GeneralStatusResponseID 4
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
  float Temperature;
  float FuelUsage;
}PK_GeneralStatusResponse_t;

#define PK_TableMemoryRequestID 5
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
}PK_TableMemoryRequest_t;

#define PK_TableMemoryDataID 6
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
  uint16_t crc;
  uint8_t data[PACKET_TABLE_MAX_SIZE];
}PK_TableMemoryData_t;

#define PK_TableMemoryAcknowledgeID 7
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
  uint32_t table;
  uint32_t tablesize;
  uint32_t offset;
  uint32_t size;
}PK_TableMemoryAcknowledge_t;

#define PK_ConfigMemoryRequestID 8
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PK_ConfigMemoryRequest_t;

#define PK_ConfigMemoryDataID 9
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
  uint16_t crc;
  uint8_t data[PACKET_CONFIG_MAX_SIZE];
}PK_ConfigMemoryData_t;

#define PK_ConfigMemoryAcknowledgeID 10
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
  uint32_t configsize;
  uint32_t offset;
  uint32_t size;
}PK_ConfigMemoryAcknowledge_t;

#define PK_SaveConfigID 11
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
}PK_SaveConfig_t;

#define PK_RestoreConfigID 12
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
}PK_RestoreConfig_t;

#define PK_SaveConfigAcknowledgeID 13
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
}PK_SaveConfigAcknowledge_t;

#define PK_RestoreConfigAcknowledgeID 14
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t ErrorCode;
}PK_RestoreConfigAcknowledge_t;

#define PK_DragStartID 15
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  float FromRPM;
  float ToRPM;
}PK_DragStart_t;

#define PK_DragUpdateRequestID 16
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
}PK_DragUpdateRequest_t;

#define PK_DragUpdateResponseID 17
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t ErrorCode;
  float FromRPM;
  float ToRPM;
  float CurrentRPM;
  float CurrentPressure;
  float CurrentLoad;
  float CurrentIgnition;
  uint32_t Time;
  uint32_t TotalPoints;
  uint8_t Started;
  uint8_t Completed;
}PK_DragUpdateResponse_t;

#define PK_DragStopID 18
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  float FromRPM;
  float ToRPM;
}PK_DragStop_t;

#define PK_DragPointRequestID 19
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint32_t PointNumber;
  float FromRPM;
  float ToRPM;
}PK_DragPointRequest_t;

#define PK_DragPointResponseID 20
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t ErrorCode;
  float FromRPM;
  float ToRPM;
  uint32_t PointNumber;
  sDragPoint Point;
}PK_DragPointResponse_t;

#define PK_DragStartAcknowledgeID 21
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t ErrorCode;
  float FromRPM;
  float ToRPM;
}PK_DragStartAcknowledge_t;

#define PK_DragStopAcknowledgeID 22
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t ErrorCode;
  float FromRPM;
  float ToRPM;
}PK_DragStopAcknowledge_t;

#define PK_PcConnectedID 23
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
}PK_PcConnected_t;

#define PK_FuelSwitchID 24
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  uint8_t FuelSwitchPos;

}PK_FuelSwitch_t;

extern int16_t PK_Copy(void * dest, void * source);

#define PACKET_C(x) extern x##_t x;

PACKET_C(PK_Ping);
PACKET_C(PK_Pong);
PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);
PACKET_C(PK_TableMemoryRequest);
PACKET_C(PK_TableMemoryData);
PACKET_C(PK_TableMemoryAcknowledge);
PACKET_C(PK_ConfigMemoryRequest);
PACKET_C(PK_ConfigMemoryData);
PACKET_C(PK_ConfigMemoryAcknowledge);
PACKET_C(PK_SaveConfig);
PACKET_C(PK_RestoreConfig);
PACKET_C(PK_SaveConfigAcknowledge);
PACKET_C(PK_RestoreConfigAcknowledge);
PACKET_C(PK_DragStart);
PACKET_C(PK_DragUpdateRequest);
PACKET_C(PK_DragUpdateResponse);
PACKET_C(PK_DragStop);
PACKET_C(PK_DragPointRequest);
PACKET_C(PK_DragPointResponse);
PACKET_C(PK_DragStartAcknowledge);
PACKET_C(PK_DragStopAcknowledge);
PACKET_C(PK_PcConnected);
PACKET_C(PK_FuelSwitch);

#undef PACKET_C


#endif /* PACKETS_H_ */
