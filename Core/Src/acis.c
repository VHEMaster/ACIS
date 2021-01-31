#include "acis.h"
#include "csps.h"
#include "map.h"
#include "delay.h"
#include "packets.h"
#include "xCommand.h"
#include "xProFIFO.h"
#include <math.h>

#define TABLE_SETUPS_MAX 4
#define TABLE_PRESSURES_MAX 32
#define TABLE_ROTATES_MAX 32
#define TABLE_TEMPERATURES_MAX 8
#define TABLE_STRING_MAX 64

typedef enum
{
  FuelNo,
  FuelPetrol,
  FuelPropane,
}eFuelType;

typedef enum
{
  ValveAllClosed,
  ValvePropane,
  ValvePetrol,
}eValveChannel;

typedef struct
{
    char name[TABLE_STRING_MAX];

    eFuelType fuel_type;
    eValveChannel valve_channel;

    float initial_ignition;

    float rpm_cutouts;
    float octane_corrector;

    uint32_t pressures_count;
    float pressures[TABLE_PRESSURES_MAX];

    uint32_t rotates_count;
    float rotates[TABLE_ROTATES_MAX];

    uint32_t idles_count;
    float idle_ignitions[TABLE_ROTATES_MAX];
    float idle_rotates[TABLE_ROTATES_MAX];

    float ignitions[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];

    float servo_acc[TABLE_TEMPERATURES_MAX];
    float servo_choke[TABLE_TEMPERATURES_MAX];

    uint32_t temperatures_count;
    float temperatures[TABLE_TEMPERATURES_MAX];
    float temperature_ignitions[TABLE_TEMPERATURES_MAX];

    uint32_t Reserved[256];
}sAcisIgnTable;

typedef struct
{
    uint8_t isEnabled;
    uint8_t isCutoutEnabled;
    uint8_t isTemperatureEnabled;
    uint8_t isEconomEnabled;
    uint8_t isAutostartEnabled;
    uint8_t isIgnitionByHall;
    uint8_t isForceTable;
    uint8_t isLearningMode;

    uint8_t switchPos1Table;
    uint8_t switchPos0Table;
    uint8_t switchPos2Table;

    uint8_t Reserved8[256 - 9];

    uint16_t EconomDisablingDelay;
    float InitialRpmThreshold;

    uint32_t Reserved32[256];
}sAcisParams;

typedef struct
{
    uint32_t tables_count;
    sAcisIgnTable tables[TABLE_SETUPS_MAX];
    sAcisParams params;
    uint32_t crc;
}sAcisConfig;

sAcisConfig acis_config;


#define IGN_OVER_TIME 500000
#define IGN_SATURATION 12000
#define IGN_PULSE 2500
#define INITIAL_TIMEOUT 100000
static uint32_t ign14_time = 0;
static uint32_t ign23_time = 0;
static uint32_t ign14_prev = 0;
static uint32_t ign23_prev = 0;
static uint8_t ign_ftime = 1;

static volatile uint32_t hall_prev = 0;
static volatile float hall_angle = 0;
static volatile float hall_error = 0;
static volatile uint8_t hall_rotates = 0;
static volatile float angle_ignite = 0;
static volatile float angle_saturate = 0;
static volatile uint8_t table_current = 0;

#define SENDING_BUFFER_SIZE 256
uint8_t buffSendingBuffer[SENDING_BUFFER_SIZE];

#define SENDING_QUEUE_SIZE 1024
uint8_t buffSendingQueue[SENDING_QUEUE_SIZE];
sProFIFO fifoSendingQueue;

extern TIM_HandleTypeDef htim4;

static int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length);

void acis_init(void)
{
  protInit(&fifoSendingQueue, buffSendingQueue, 1, SENDING_QUEUE_SIZE);
  HAL_TIM_Base_Start_IT(&htim4);
}

static inline void acis_ignite_14(void)
{
  HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
  ign14_prev = ign14_time;
  ign14_time = Delay_Tick;
  ign_ftime = 0;
}

static inline void acis_ignite_23(void)
{
  HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
  ign23_prev = ign23_time;
  ign23_time = Delay_Tick;
  ign_ftime = 0;
}

static inline void acis_saturate_14(void)
{
  HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
  ign_ftime = 0;
}

static inline void acis_saturate_23(void)
{
  HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
  ign_ftime = 0;
}

static inline void acis_ignite(uint8_t index)
{
  if(index == 0)
    acis_ignite_14();
  else if(index == 1)
    acis_ignite_23();
}

static inline void acis_saturate(uint8_t index)
{
  if(index == 0)
    acis_saturate_14();
  else if(index == 1)
    acis_saturate_23();
}

static inline void acis_ignition_loop(void)
{
  uint32_t now = Delay_Tick;
  float rpm = csps_getrpm();
  uint8_t rotates = csps_isrotates() || hall_rotates;
  if(!ign_ftime && rotates)
  {
    if(DelayDiff(now, ign14_time) >= IGN_OVER_TIME)
      HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
    else if(acis_config.params.isIgnitionByHall)
    {
      if(rpm < 300.0f)
      {
        if(DelayDiff(now, ign14_time) >= DelayDiff(ign14_time, ign14_prev) * 96 / 128)
          HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
      }
      else if(DelayDiff(ign14_time, ign14_prev) > 15000)
      {
        if((int32_t)DelayDiff(ign14_time, ign14_prev) - (int32_t)DelayDiff(now, ign14_time) < 11719)
        {
          HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
        }
      }
      else if(DelayDiff(now, ign14_time) >= DelayDiff(ign14_time, ign14_prev) * 28 / 128)
        HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
    }

    if(DelayDiff(now, ign23_time) >= IGN_OVER_TIME)
      HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
    else if(acis_config.params.isIgnitionByHall)
    {
      if(rpm < 300.0f)
      {
        if(DelayDiff(now, ign23_time) >= DelayDiff(ign23_time, ign23_prev) * 96 / 128)
          HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
      }
      else if(rpm > 500.0f && DelayDiff(ign23_time, ign23_prev) > 15000)
      {
        if((int32_t)DelayDiff(ign23_time, ign23_prev) - (int32_t)DelayDiff(now, ign23_time) < 11719)
        {
          HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
        }
      }
      else if(DelayDiff(now, ign23_time) >= DelayDiff(ign23_time, ign23_prev) * 28 / 128)
        HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
    }
  }
  else
  {
    HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
  }
}

inline void acis_hall_exti(void)
{
  uint32_t now = Delay_Tick;
  uint8_t hall_cylinders = 0;
  hall_prev = now;
  float angle = 0.0f;
  float angle14 = csps_getangle14();
  float rpm = csps_getrpm();
  float pressure = map_getpressure();
  float angle23 = csps_getangle23from14(angle14);
  float cspsfound = csps_isfound();

  if(angle23 > 90.0f || angle23 <= -90.0f)
  {
      hall_cylinders = 2;
      angle = angle14;
  }
  if(angle14 > 90.0f || angle14 <= -90.0f)
  {
    if(hall_cylinders == 0)
    {
      hall_cylinders = 1;
      angle = angle23;
    }
    else
    {
      hall_cylinders = 0;
      hall_error += 1.0f;
    }
  }
  hall_angle = angle;

  if(cspsfound && acis_config.params.isIgnitionByHall)
  {
    if(hall_cylinders == 1)
    {
      acis_ignite_14();
    }
    else if(hall_cylinders == 2)
    {
      acis_ignite_23();
    }
  }
  hall_rotates = 1;
}

static inline void acis_hall_loop(void)
{
  static uint32_t last_error_null = 0;
  uint32_t prev = hall_prev;
  uint32_t now = Delay_Tick;
  if(DelayDiff(now, last_error_null) > 100000)
  {
    hall_error *= 0.95;
    last_error_null = now;
  }
  if(DelayDiff(now, prev) > 600000)
  {
    hall_rotates = 0;
  }
}

static float CalculateIgnition(void)
{
  float rpm = csps_getrpm();
  float pressure = map_getpressure();
  float temperature = 80.0f;
  uint8_t isIdle = HAL_GPIO_ReadPin(SENS_ACC_GPIO_Port, SENS_ACC_Pin) == GPIO_PIN_SET;


  static uint32_t lastRotated = 0x80000000;
  static uint8_t isInitial = 1;
  uint32_t now = Delay_Tick;
  float angle = 0.0f;
  float angle_1, angle_2;
  float mult, mult_rpm, mult_press,
      temprpm1 = 0.0f,temprpm2 = 0.0f,
      temppress1 = 0.0f,temppress2 = 0.0f,
      temptemp1 = 0.0f,temptemp2 = 0.0f,
      tempign1 = 0.0f, tempign2 = 0.0f,
      tempign11 = 0.0f, tempign12 = 0.0f,
      tempign21 = 0.0f, tempign22 = 0.0f;
  uint32_t rpmindex1 = 0, rpmindex2 = 0,
      pressindex1 = 0, pressindex2 = 0,
      tempindex1 = 0, tempindex2 = 0;

  sAcisIgnTable * table = NULL;
  int table_num = table_current;

  if(table_num < acis_config.tables_count || table_num < TABLE_SETUPS_MAX)
    table = &acis_config.tables[table_num];

  if(rpm > 400.0f)
  {
    isInitial = 0;
    lastRotated = now;
  }
  else if(rpm < 10.0f)
  {
    if(DelayDiff(now, lastRotated) < INITIAL_TIMEOUT)
    {
      isInitial = 1;
    }
  }

  if(table)
  {
    angle = table->initial_ignition;
    if(!isInitial && rpm > 10.0f)
    {
      if(isIdle)
      {
        if(table->idles_count == 0)
          angle = table->initial_ignition;
        else if(table->idles_count == 1)
          angle = table->idle_ignitions[0];
        else
        {
          if(rpm <= table->idle_rotates[0])
          {
            angle = table->idle_ignitions[0];
          }
          else if(rpm >= table->idle_rotates[table->rotates_count-1])
          {
            rpmindex1 = table->idles_count-2;
            rpmindex2 = table->idles_count-1;
            temprpm1 = table->idle_rotates[rpmindex1];
            temprpm2 = table->idle_rotates[rpmindex2];
          }
          else
          {
            for(int i = 1; i < table->idles_count; i++)
            {
              temprpm1 = table->idle_rotates[i-1];
              temprpm2 = table->idle_rotates[i];
              if(temprpm1 < rpm && temprpm2 > rpm)
              {
                rpmindex1 = i-1;
                rpmindex2 = i;
                break;
              }
              temprpm1 = 0.0f;
              temprpm2 = 0.0f;
            }
          }
          if(temprpm1 != 0.0f || temprpm2 != 0.0f)
          {
            tempign1 = table->idle_ignitions[rpmindex1];
            tempign2 = table->idle_ignitions[rpmindex2];
            mult = (rpm - temprpm1) / (temprpm2 - temprpm1);
            angle = (tempign2 - tempign1) * mult + tempign1;
          }

        }
      }
      else
      {
        if(table->pressures_count != 0 && table->rotates_count != 0)
        {
          if(table->pressures_count == 1)
          {
            pressindex1 = 0;
            pressindex2 = 0;
            temppress1 = table->pressures[0];
            temppress2 = table->pressures[0];
          }
          else if(pressure >= table->pressures[0])
          {
            pressindex1 = 0;
            pressindex2 = 1;
            temppress1 = table->pressures[pressindex1];
            temppress2 = table->pressures[pressindex2];
          }
          else if(pressure <= table->pressures[table->pressures_count - 1])
          {
            pressindex1 = table->pressures_count - 2;
            pressindex2 = table->pressures_count - 1;
            temppress1 = table->pressures[pressindex1];
            temppress2 = table->pressures[pressindex2];
          }
          else
          {
            for(int i = 1; i < table->pressures_count; i++)
            {
              temppress1 = table->pressures[i-1];
              temppress2 = table->pressures[i];
              if(temppress1 > pressure && temppress2 < pressure)
              {
                pressindex1 = i-1;
                pressindex2 = i;
                break;
              }
              temppress1 = 0.0f;
              temppress2 = 0.0f;
            }
          }

          if(table->rotates_count == 1)
          {
            rpmindex1 = 0;
            rpmindex2 = 0;
            temprpm1 = table->rotates[0];
            temprpm2 = table->rotates[0];
          }
          else if(rpm <= table->rotates[0])
          {
            rpmindex1 = 0;
            rpmindex2 = 1;
            temprpm1 = table->rotates[rpmindex1];
            temprpm2 = table->rotates[rpmindex2];
          }
          else if(rpm >= table->rotates[table->rotates_count - 1])
          {
            rpmindex1 = table->rotates_count - 2;
            rpmindex2 = table->rotates_count - 1;
            temprpm1 = table->rotates[rpmindex1];
            temprpm2 = table->rotates[rpmindex2];
          }
          else
          {
            for(int i = 1; i < table->rotates_count; i++)
            {
              temprpm1 = table->rotates[i-1];
              temprpm2 = table->rotates[i];
              if(temprpm1 < rpm && temprpm2 > rpm)
              {
                rpmindex1 = i-1;
                rpmindex2 = i;
                break;
              }
              temprpm1 = 0.0f;
              temprpm2 = 0.0f;
            }
          }

          if((temprpm1 != 0.0f || temprpm2 != 0.0f) && (temppress1 != 0.0f || temppress2 != 0.0f))
          {
            tempign11 = table->ignitions[pressindex1][rpmindex1];
            tempign12 = table->ignitions[pressindex1][rpmindex2];
            tempign21 = table->ignitions[pressindex2][rpmindex1];
            tempign22 = table->ignitions[pressindex2][rpmindex2];

            mult_rpm = (rpm - temprpm1) / (temprpm2 - temprpm1);
            mult_press = (pressure - temppress1) / (temppress2 - temppress1);

            angle_1 = (tempign12 - tempign11) * mult_rpm + tempign11;
            angle_2 = (tempign22 - tempign21) * mult_rpm + tempign21;

            angle = (angle_2 - angle_1) * mult_press + angle_1;
          }


        }
      }

      if(table->temperatures_count >= 2)
      {
        if(temperature <= table->temperatures[0])
          angle += table->temperature_ignitions[0];
        else if(temperature >= table->temperatures[table->temperatures_count - 1])
        {
          tempindex1 = table->temperatures_count-2;
          tempindex2 = table->temperatures_count-1;
          temptemp1 = table->temperatures[tempindex1];
          temptemp2 = table->temperatures[tempindex2];
        }
        else
        {
          for(int i = 1; i < table->temperatures_count; i++)
          {
            temptemp1 = table->temperatures[i-1];
            temptemp2 = table->temperatures[i];
            if(temptemp1 < temperature && temptemp2 > temperature)
            {
              tempindex1 = i-1;
              tempindex2 = i;
              break;
            }
            temptemp1 = 0.0f;
            temptemp2 = 0.0f;
          }
        }
        if(temptemp1 != 0.0f || temptemp2 != 0.0f)
        {
          tempign1 = table->temperature_ignitions[tempindex1];
          tempign2 = table->temperature_ignitions[tempindex2];
          mult = (temperature - temptemp1) / (temptemp2 - temptemp1);
          angle += (tempign2 - tempign1) * mult + tempign1;
        }
      }
    }
    angle += table->octane_corrector;
  }
  return angle;
}

//float ffangles[2][2048];
//float ffanglbi[2][2048];
//uint8_t ffsat[2][2048];
//uint8_t ffign[2][2048];
//uint16_t ffcnt[2] = {0,0};

inline void acis_loop_irq(void)
{
  if(acis_config.params.isIgnitionByHall)
    return;

  static float oldanglesbeforeignite[2] = {0,0};
  static uint8_t saturated[2] = { 1,1 };
  static uint8_t ignited[2] = { 1,1 };
  float angle[2] = { 0.0f, 0.0f };
  float anglesbeforeignite[2];
  angle[0] = csps_getangle14();
  angle[1] = csps_getangle23from14(angle[0]);


  float uspa = csps_getuspa();
  float period = csps_getperiod();
  float time_sat = IGN_SATURATION;
  float ignite = 0;

  float found = csps_isfound();



  if(period < IGN_SATURATION + IGN_PULSE)
  {
    time_sat = period * ((float)IGN_SATURATION / (float)(IGN_SATURATION + IGN_PULSE));
  }

  float saturate = time_sat / uspa;

  ignite = angle_ignite;
  angle_saturate = saturate;


  if(found)
  {
    for(int i = 0; i < 2; i++)
    {


      if(angle[i] < -ignite)
        anglesbeforeignite[i] = -angle[i] - ignite;
      else
        anglesbeforeignite[i] = 360.0f - angle[i] - ignite;

      if(anglesbeforeignite[i] - oldanglesbeforeignite[i] > 0.0f && anglesbeforeignite[i] - oldanglesbeforeignite[i] < 180.0f)
        anglesbeforeignite[i] = oldanglesbeforeignite[i];

      if(anglesbeforeignite[i] - saturate < 0.0f)
      {
        if(!saturated[i] && !ignited[i])
        {
          saturated[i] = 1;
          acis_saturate(i);
        }
      }

      if(oldanglesbeforeignite[i] - anglesbeforeignite[i] < -1.0f)
      {
        if(!ignited[i] && saturated[i])
        {
          ignited[i] = 1;
          saturated[i] = 0;
          acis_ignite(i);
        }
      }
      else ignited[i] = 0;

      //ffangles[i][ffcnt[i]] = angle[i];
      //ffanglbi[i][ffcnt[i]] = anglesbeforeignite[i];
      //ffsat[i][ffcnt[i]] = saturated[i];
      //ffign[i][ffcnt[i]] = ignited[i];
      //if(++ffcnt[i] >= 2048) ffcnt[i] = 0;

      oldanglesbeforeignite[i] = anglesbeforeignite[i];
    }
  }
  else
  {
    angle_ignite = 0;
  }

  acis_hall_loop();

  acis_ignition_loop();
}
inline void acis_loop(void)
{
  static uint8_t sending = 0;
  static uint8_t destination = 0;
  static uint8_t size = 0;
  uint8_t * pnt;
  do
  {
    if(!sending && protGetSize(&fifoSendingQueue) > 4)
    {
      protLook(&fifoSendingQueue,1,&size);
      protLook(&fifoSendingQueue,2,&destination);
      if(protGetSize(&fifoSendingQueue) >= size)
      {
        pnt = buffSendingBuffer;
        for(int i = 0; i < size; i++)
          protPull(&fifoSendingQueue, pnt++);
        if(destination)
          sending = 1;
      }
    }
    if(sending)
    {
      if(acis_send_command(destination, buffSendingBuffer, size) != 0)
      {
        sending = 0;
        continue;
      }
    }
  } while(0);

  if(!acis_config.params.isIgnitionByHall)
  {
    angle_ignite = CalculateIgnition();
  }
  else angle_ignite = 0.0f;
}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  switch(msgBuf[0])
  {
    case PK_GeneralStatusRequestID :
      //PK_Copy(&PK_GeneralStatusRequest, msgBuf);
      PK_GeneralStatusResponse.Destination = xChaSrc;
      PK_GeneralStatusResponse.RPM = csps_getrpm();
      PK_GeneralStatusResponse.Pressure = map_getpressure();
      PK_GeneralStatusResponse.Load = map_getpressure() / 110000.0f * 100.0f;
      PK_GeneralStatusResponse.IgnitionAngle = angle_ignite;
      protPushSequence(&fifoSendingQueue, &PK_GeneralStatusResponse, sizeof(PK_GeneralStatusResponse));
      break;
    default:
      break;
  }
}

inline int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length)
{
  return xSender(xChaDst, (uint8_t*)msgBuf, length);
}


