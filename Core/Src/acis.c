#include "acis.h"
#include "csps.h"
#include "map.h"
#include "delay.h"
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
    float idles[TABLE_ROTATES_MAX];

    float ignitions[TABLE_PRESSURES_MAX][TABLE_ROTATES_MAX];

    float servo_acc[TABLE_TEMPERATURES_MAX];
    float servo_choke[TABLE_TEMPERATURES_MAX];

    uint32_t temperatures_count;
    float temperatures[TABLE_TEMPERATURES_MAX];

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

sAcisConfig acis_config;// __attribute__((section(".ItcmRamSection")));

extern TIM_HandleTypeDef htim4;

void acis_init(void)
{
  HAL_TIM_Base_Start_IT(&htim4);
}

#define IGN_OVER_TIME 500000
#define IGN_SATURATION 12000
#define IGN_PULSE 2000
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

inline void acis_loop(void)
{
  static float oldanglesbeforeignite[2] = {0,0};
  static uint8_t saturated[2] = { 1,1 };
  static uint8_t ignited[2] = { 1,1 };
  float angle[2] = { 0.0f, 0.0f };
  float anglesbeforeignite[2];
  static float oldsaturate = 0;
  angle[0] = csps_getangle14();
  angle[1] = csps_getangle23from14(angle[0]);
  sAcisIgnTable * table = NULL;

  float rpm = csps_getrpm();
  float uspa = csps_getuspa();
  float period = csps_getperiod();
  float pressure = map_getpressure();
  float time_sat = IGN_SATURATION;

  float found = csps_isfound();
  uint32_t now = Delay_Tick;

  int table_num = table_current;

  if(table_num < acis_config.tables_count || table_num < TABLE_SETUPS_MAX)
    table = &acis_config.tables[table_num];


  float ignite = 0.0f;

  if(period < IGN_SATURATION + IGN_PULSE)
  {
    time_sat = period * ((float)IGN_SATURATION / (float)(IGN_SATURATION + IGN_PULSE));
  }

  if(table)
  {

  }

  float saturate = time_sat / uspa;

  angle_ignite = ignite;
  angle_saturate = saturate;


  if(found)
  {
    for(int i = 0; i < 2; i++)
    {

      if(angle[i] < -ignite)
        anglesbeforeignite[i] = -angle[i] - ignite;
      else
        anglesbeforeignite[i] = 360.0f - angle[i] - ignite;

      if(anglesbeforeignite[i] - saturate < 0.0f)
      {
        if(!saturated[i])
        {
          saturated[i] = 1;
          acis_saturate(i);
        }
      }
      else saturated[i] = 0;

      if(oldanglesbeforeignite[i] - anglesbeforeignite[i] < 0.0f)
      {
        if(!ignited[i])
        {
          ignited[i] = 1;
          acis_ignite(i);
        }
      }
      else ignited[i] = 0;

      oldanglesbeforeignite[i] = anglesbeforeignite[i];

    }
  }
  else
  {
    angle_ignite = 0;
  }

  oldsaturate = saturate;

  acis_hall_loop();

  acis_ignition_loop();
}


