#include "acis.h"
#include "csps.h"
#include "map.h"
#include "delay.h"
#include "packets.h"
#include "xCommand.h"
#include "xProFIFO.h"
#include "config.h"
#include <math.h>
#include <string.h>

sAcisConfig acis_config;

#define IGN_OVER_TIME 600000
#define IGN_SATURATION 1800
#define IGN_PULSE 2500
#define INITIAL_TIMEOUT 100000
static uint32_t ign14_sat = 0;
static uint32_t ign23_sat = 0;
static uint32_t ign14_time = 0;
static uint32_t ign23_time = 0;
static uint32_t ign14_prev = 0;
static uint32_t ign23_prev = 0;
static uint8_t ign_ftime = 1;

static volatile uint32_t hall_prev = 0;
static volatile float hall_angle = 0;
static volatile float hall_error = 0;
static volatile uint8_t hall_rotates = 0;
static volatile float angle_time = 0;
static volatile float angle_ignite = 0;
static volatile float angle_saturate = 0;
static volatile uint8_t table_current = 0;
static volatile uint8_t valve_current = 0;
static volatile float power_voltage = 0;
static volatile int8_t switch_fuel_pos = 0;
static volatile HAL_StatusTypeDef StatusInit = HAL_ERROR;

static volatile uint8_t saturated_14 = 0;
static volatile uint8_t saturated_23 = 0;

#define SENDING_BUFFER_SIZE 256
uint8_t buffSendingBuffer[SENDING_BUFFER_SIZE];

#define SENDING_QUEUE_SIZE 1024
uint8_t buffSendingQueue[SENDING_QUEUE_SIZE];
sProFIFO fifoSendingQueue;

extern TIM_HandleTypeDef htim4;

volatile uint32_t TIME1 = 0;
volatile uint32_t TIME2 = 0;
volatile uint32_t TIME3 = 0;

volatile uint32_t TIME11 = 0;
volatile uint32_t TIME22 = 0;
volatile uint32_t TIME33 = 0;

static int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length);

void acis_init(void)
{
  HAL_StatusTypeDef status = HAL_BUSY;
  protInit(&fifoSendingQueue, buffSendingQueue, 1, SENDING_QUEUE_SIZE);

  //do
  //{
  //  status = config_load(&acis_config);
  //} while(status == HAL_BUSY);

  StatusInit = status;

  //if(status != HAL_OK)
  //{
    config_default(&acis_config);
  //  do
  //  {
  //    status = config_save(&acis_config);
  //  } while(status == HAL_BUSY);
  //}

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_SET);
}

void acis_deinitIfNeed(void)
{
  if(HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_SET)
    HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_RESET);
  else HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_SET);
}

static inline void acis_ignite_14(void)
{
  ign14_prev = ign14_time;
  ign14_time = Delay_Tick;
  HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
  ign_ftime = 0;
  saturated_14 = 0;
}

static inline void acis_ignite_23(void)
{
  ign23_prev = ign23_time;
  ign23_time = Delay_Tick;
  HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
  ign_ftime = 0;
  saturated_23 = 0;
}

static inline void acis_saturate_14(void)
{
  ign14_sat = Delay_Tick;
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;
  if(isIgn)
    HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
  ign_ftime = 0;
  saturated_14 = 1;
}

static inline void acis_saturate_23(void)
{
  ign23_sat = Delay_Tick;
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;
  if(isIgn)
    HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
  ign_ftime = 0;
  saturated_23 = 1;
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
  TIME11 = Delay_Tick;
  float rpm = csps_getrpm();
  uint8_t rotates = csps_isrotates() || hall_rotates;
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;

  if(isIgn && !ign_ftime && rotates)
  {
    uint8_t t_saturated_14 = saturated_14;
    uint32_t t_ign14_sat = ign14_sat;
    uint32_t t_ign14_time = ign14_time;
    uint32_t t_ign14_prev = ign14_prev;
    uint32_t now = Delay_Tick;
    if(DelayDiff(now, t_ign14_time) >= IGN_OVER_TIME && DelayDiff(now, t_ign14_sat) >= IGN_OVER_TIME)
    {
      HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
      saturated_14 = 0;
    }
    else if(!t_saturated_14 && acis_config.params.isIgnitionByHall)
    {
      if(rpm < 300.0f)
      {
        if(DelayDiff(now, t_ign14_time) >= DelayDiff(t_ign14_time, t_ign14_prev) * 96 / 128)
        {
          HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
          saturated_14 = 1;
        }
      }
      else if(DelayDiff(t_ign14_time, t_ign14_prev) > 15000)
      {
        if((int32_t)DelayDiff(t_ign14_time, t_ign14_prev) - (int32_t)DelayDiff(now, t_ign14_time) < 11719)
        {
          HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
          saturated_14 = 1;
        }
      }
      else if(DelayDiff(now, t_ign14_time) >= DelayDiff(t_ign14_time, t_ign14_prev) * 64 / 128)
      {
        HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_RESET);
        saturated_14 = 1;
      }
    }

    uint8_t t_saturated_23 = saturated_23;
    uint32_t t_ign23_sat = ign23_sat;
    uint32_t t_ign23_time = ign23_time;
    uint32_t t_ign23_prev = ign23_prev;
    now = Delay_Tick;
    if(DelayDiff(now, t_ign23_time) >= IGN_OVER_TIME && DelayDiff(now, t_ign23_sat) >= IGN_OVER_TIME)
    {
      HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
      saturated_23 = 0;
    }
    else if(!t_saturated_23 && acis_config.params.isIgnitionByHall)
    {
      if(rpm < 300.0f)
      {
        if(DelayDiff(now, t_ign23_time) >= DelayDiff(t_ign23_time, t_ign23_prev) * 96 / 128)
        {
          HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
          saturated_23 = 1;
        }
      }
      else if(rpm > 500.0f && DelayDiff(t_ign23_time, t_ign23_prev) > 15000)
      {
        if((int32_t)DelayDiff(t_ign23_time, t_ign23_prev) - (int32_t)DelayDiff(now, t_ign23_time) < 11719)
        {
          HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
          saturated_23 = 1;
        }
      }
      else if(DelayDiff(now, t_ign23_time) >= DelayDiff(t_ign23_time, t_ign23_prev) * 64 / 128)
      {
        HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_RESET);
        saturated_23 = 1;
      }
    }
  }
  else
  {
    HAL_GPIO_WritePin(IGN_14_GPIO_Port, IGN_14_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IGN_23_GPIO_Port, IGN_23_Pin, GPIO_PIN_SET);
    saturated_14 = 0;
    saturated_23 = 0;
  }
  TIME1 = Delay_Tick - TIME11;
}

inline void acis_hall_exti(void)
{
  uint32_t now = Delay_Tick;
  uint8_t hall_cylinders = 0;
  hall_prev = now;
  float angle = 0.0f;
  float angle14 = csps_getangle14();
  float angle23 = csps_getangle23from14(angle14);
  float cspsfound = csps_isfound();

  if(angle14 < 90.0f && angle14 >= -90.0f)
  {
      hall_cylinders = 1;
      angle = angle14;
  }
  else
  {
    hall_cylinders = 2;
    angle = angle23;
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
  if(DelayDiff(now, last_error_null) > 10000)
  {
    hall_error *= 0.95;
    last_error_null = now;
  }
  if(DelayDiff(now, prev) > 600000)
  {
    if((acis_config.params.isHallLearningMode || acis_config.params.isIgnitionByHall) && csps_isfound())
      hall_error = 3.0f;
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
            if(temprpm1 != temprpm2)
            {
              tempign1 = table->idle_ignitions[rpmindex1];
              tempign2 = table->idle_ignitions[rpmindex2];
              mult = (rpm - temprpm1) / (temprpm2 - temprpm1);
              angle = (tempign2 - tempign1) * mult + tempign1;
            }
            else angle = (tempign1 + tempign2) / 2.0f;
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
          else if(pressure <= table->pressures[0])
          {
            pressindex1 = 0;
            pressindex2 = 1;
            temppress1 = table->pressures[pressindex1];
            temppress2 = table->pressures[pressindex2];
          }
          else if(pressure >= table->pressures[table->pressures_count - 1])
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
              if(temppress1 < pressure && temppress2 > pressure)
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

            if(temprpm2 != temprpm1 && temppress1 != temppress2)
            {
              mult_rpm = (rpm - temprpm1) / (temprpm2 - temprpm1);
              mult_press = (pressure - temppress1) / (temppress2 - temppress1);

              angle_1 = (tempign12 - tempign11) * mult_rpm + tempign11;
              angle_2 = (tempign22 - tempign21) * mult_rpm + tempign21;

              angle = (angle_2 - angle_1) * mult_press + angle_1;
            }
            else if(temprpm2 == temprpm1 && temppress1 != temppress2)
            {
              mult_press = (pressure - temppress1) / (temppress2 - temppress1);
              mult_rpm = 1.0f;
              angle = (tempign21 - tempign11) * mult_press + tempign11;
            }
            else if(temprpm2 != temprpm1 && temppress1 == temppress2)
            {
              mult_rpm = (rpm - temprpm1) / (temprpm2 - temprpm1);
              mult_press = 1.0f;
              angle = (tempign12 - tempign11) * mult_rpm + tempign11;
            }
            else angle = (tempign11 + tempign12 + tempign21 + tempign22) / 4.0f;

          }
        }
      }

      if(acis_config.params.isTemperatureEnabled)
      {
        if(table->temperatures_count == 1)
        {
          angle += table->temperature_ignitions[0];
        }
        else if(table->temperatures_count >= 2)
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
            if(temptemp1 != temptemp2)
            {
              tempign1 = table->temperature_ignitions[tempindex1];
              tempign2 = table->temperature_ignitions[tempindex2];
              mult = (temperature - temptemp1) / (temptemp2 - temptemp1);
              angle += (tempign2 - tempign1) * mult + tempign1;
            }
            else angle += (tempign1 + tempign2) / 2.0f;
          }
        }
      }
    }
    angle += table->octane_corrector;
    if(angle > 60.0f) angle = 60.0f;
    if(angle < -60.0f) angle = -60.0f;
  }
  return angle;
}

static void LearnIgnition(void)
{
  if(!hall_rotates)
    return;

  float rpm = csps_getrpm();
  float angle_needed = hall_angle;
  float pressure = map_getpressure();
  uint8_t isIdle = HAL_GPIO_ReadPin(SENS_ACC_GPIO_Port, SENS_ACC_Pin) == GPIO_PIN_SET;

  static uint32_t lastRotated = 0x80000000;
  static uint32_t lastAccepted = 0;
  static uint8_t isInitial = 1;
  uint32_t now = Delay_Tick;
  float angle = 0.0f;
  float angle_1, angle_2;
  float mult, diff, mult_rpm, mult_press,
      temprpm1 = 0.0f,temprpm2 = 0.0f,
      temppress1 = 0.0f,temppress2 = 0.0f,
      tempign1 = 0.0f, tempign2 = 0.0f,
      tempign11 = 0.0f, tempign12 = 0.0f,
      tempign21 = 0.0f, tempign22 = 0.0f;
  uint32_t rpmindex1 = 0, rpmindex2 = 0,
      pressindex1 = 0, pressindex2 = 0;

  sAcisIgnTable * table = NULL;
  int table_num = table_current;

  if(table_num < acis_config.tables_count || table_num < TABLE_SETUPS_MAX)
    table = &acis_config.tables[table_num];

  if(rpm > 100.0f)
  {
    isInitial = 0;
    lastRotated = now;
  }
  else if(rpm < 10.0f)
  {
    lastAccepted = now;
    if(DelayDiff(now, lastRotated) < INITIAL_TIMEOUT)
    {
      isInitial = 1;
    }
  }

  float timediff = DelayDiff(now, lastAccepted);
  if(timediff < 10000)
    return;

  float accept_coff = timediff / 10000000.0f;
  float accept_coff_inv = 1.0f - accept_coff;
  lastAccepted = now;

  if(table)
  {
    if(!isInitial)
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
            if(tempign1 != tempign2)
            {
              mult = (rpm - temprpm1) / (temprpm2 - temprpm1);
              angle = (tempign2 - tempign1) * mult + tempign1;

              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;
              diff = angle_needed - angle;

              table->idle_ignitions[rpmindex1] = tempign1 + diff * (1.0f - mult);
              table->idle_ignitions[rpmindex2] = tempign2 + diff * mult;
            }
            else
            {
              angle = tempign1;
              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;

              table->idle_ignitions[rpmindex1] = angle_needed;
              table->idle_ignitions[rpmindex2] = angle_needed;
            }
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
          else if(pressure <= table->pressures[0])
          {
            pressindex1 = 0;
            pressindex2 = 1;
            temppress1 = table->pressures[pressindex1];
            temppress2 = table->pressures[pressindex2];
          }
          else if(pressure >= table->pressures[table->pressures_count - 1])
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
              if(temppress1 < pressure && temppress2 > pressure)
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

            if(temprpm2 != temprpm1 && temppress1 != temppress2)
            {
              mult_rpm = (rpm - temprpm1) / (temprpm2 - temprpm1);
              mult_press = (pressure - temppress1) / (temppress2 - temppress1);

              angle_1 = (tempign12 - tempign11) * mult_rpm + tempign11;
              angle_2 = (tempign22 - tempign21) * mult_rpm + tempign21;

              angle = (angle_2 - angle_1) * mult_press + angle_1;
              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;
              diff = angle_needed - angle;

              table->ignitions[pressindex1][rpmindex1] = tempign11 + diff * (1.0f - mult_rpm) * (1.0f - mult_press);
              table->ignitions[pressindex1][rpmindex2] = tempign12 + diff * mult_rpm * (1.0f - mult_press);
              table->ignitions[pressindex2][rpmindex1] = tempign21 + diff * (1.0f - mult_rpm) * mult_press;
              table->ignitions[pressindex2][rpmindex2] = tempign22 + diff * mult_rpm * mult_press;

            }
            else if(temprpm2 == temprpm1 && temppress1 != temppress2)
            {
              mult_press = (pressure - temppress1) / (temppress2 - temppress1);
              mult_rpm = 1.0f;
              angle = (tempign21 - tempign11) * mult_press + tempign11;

              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;
              diff = angle_needed - angle;

              table->ignitions[pressindex1][rpmindex1] = tempign11 + diff * (1.0f - mult_press);
              table->ignitions[pressindex1][rpmindex2] = tempign12 + diff * (1.0f - mult_press);
              table->ignitions[pressindex2][rpmindex1] = tempign21 + diff * mult_press;
              table->ignitions[pressindex2][rpmindex2] = tempign22 + diff * mult_press;
            }
            else if(temprpm2 != temprpm1 && temppress1 == temppress2)
            {
              mult_rpm = (rpm - temprpm1) / (temprpm2 - temprpm1);
              mult_press = 1.0f;
              angle = (tempign12 - tempign11) * mult_rpm + tempign11;

              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;
              diff = angle_needed - angle;

              table->ignitions[pressindex1][rpmindex1] = tempign11 + diff * (1.0f - mult_rpm);
              table->ignitions[pressindex1][rpmindex2] = tempign12 + diff * mult_rpm;
              table->ignitions[pressindex2][rpmindex1] = tempign21 + diff * (1.0f - mult_rpm);
              table->ignitions[pressindex2][rpmindex2] = tempign22 + diff * mult_rpm;
            }
            else
            {
              angle = tempign11;
              angle_needed = angle * accept_coff_inv + accept_coff * angle_needed;

              table->ignitions[pressindex1][rpmindex1] = angle_needed;
              table->ignitions[pressindex1][rpmindex2] = angle_needed;
              table->ignitions[pressindex2][rpmindex1] = angle_needed;
              table->ignitions[pressindex2][rpmindex2] = angle_needed;
            }
          }
        }
      }
    }
  }
}

inline void acis_loop_irq(void)
{
  TIME22 = Delay_Tick;
  static float oldanglesbeforeignite[2] = {0,0};
  static uint8_t saturated[2] = { 1,1 };
  static uint8_t ignited[2] = { 1,1 };
  static uint8_t start_ign_state[2] = {0,0};
  static uint8_t start_ign_allow[2] = {0,0};
  static uint32_t start_ign_last[2] = {0,0};
  uint32_t now = Delay_Tick;
  float angle[2] = { 0.0f, 0.0f };
  float anglesbeforeignite[2];
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;
  angle[0] = csps_getangle14();
  angle[1] = csps_getangle23from14(angle[0]);

  if(!isIgn)
    return;

  if(acis_config.params.isEconOutAsStrobe)
  {
    if(csps_isrotates() && angle[0] > -1.0f && angle[0] < 1.0f)
    {
      HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_RESET);
    }
  }

  if(acis_config.params.isIgnitionByHall)
    return;

  float rpm = csps_getrpm();
  float uspa = csps_getuspa();
  float period = csps_getperiod();
  float time_sat = IGN_SATURATION;

  if(rpm < 400) time_sat = 7000;
  else if(rpm < 700) time_sat = 6000;
  else if(rpm < 1000) time_sat = 5000;
  else if(rpm < 1300) time_sat = 4000;
  else if(rpm < 1500) time_sat = 3000;
  float ignite = 0;

  float found = csps_isfound();


  if(found)
  {
    if(period < IGN_SATURATION + IGN_PULSE)
    {
      time_sat = period * ((float)IGN_SATURATION / (float)(IGN_SATURATION + IGN_PULSE));
    }

    float saturate = time_sat / uspa;

    ignite = angle_ignite;
    angle_saturate = saturate;

    angle_time = ignite * uspa;

    for(int i = 0; i < 2; i++)
    {

      if(angle[i] < -ignite)
        anglesbeforeignite[i] = -angle[i] - ignite;
      else
        anglesbeforeignite[i] = 360.0f - angle[i] - ignite;

      if(anglesbeforeignite[i] - oldanglesbeforeignite[i] > 0.0f && anglesbeforeignite[i] - oldanglesbeforeignite[i] < 180.0f)
        anglesbeforeignite[i] = oldanglesbeforeignite[i];

      if(rpm < 350.0f && start_ign_allow[i] && angle[i] < 80.0f)
      {
        if(start_ign_state[i] && DelayDiff(now, start_ign_last[i]) > 4000)
        {
          acis_ignite(i);
          start_ign_state[i] = 0;
          start_ign_last[i] = now;
        }
        else if(!start_ign_state[i] && DelayDiff(now, start_ign_last[i]) > 1000)
        {
          acis_saturate(i);
          start_ign_state[i] = 1;
          start_ign_last[i] = now;
        }
      }
      else
      {
        if(start_ign_state[i])
        {
          start_ign_state[i] = 0;
          acis_ignite(i);
        }
        start_ign_allow[i] = 0;
      }

      if(anglesbeforeignite[i] - saturate < 0.0f)
      {
        if(!saturated[i] && !ignited[i])
        {
          saturated[i] = 1;
          start_ign_allow[i] = 0;
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
          if(rpm < 350.0f)
          {
            start_ign_last[i] = now;
            start_ign_state[i] = 0;
            start_ign_allow[i] = 1;
          }
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
  TIME2 = Delay_Tick - TIME22;
}

inline void acis_loop(void)
{
  TIME33 = Delay_Tick;

  static uint8_t sending = 0;
  static uint8_t destination = 0;
  static uint8_t size = 0;
  static uint32_t rpm_last = 0;
  static eValveChannel valve_old = ValveAllClosed;
  uint32_t now = Delay_Tick;
  float rpm = csps_getrpm();
  uint8_t isIdle = HAL_GPIO_ReadPin(SENS_ACC_GPIO_Port,SENS_ACC_Pin) == GPIO_PIN_SET;
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;
  uint8_t * pnt;
  uint8_t status;
  uint8_t table = 0;
  uint8_t fuelsw = switch_fuel_pos;

  if(rpm_last == 0 || rpm > 10.0f) rpm_last = now;

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
      status = acis_send_command(destination, buffSendingBuffer, size);
      if(status != 0)
      {
        sending = 0;
        continue;
      }
    }
  } while(0);

  if(isIgn && !acis_config.params.isIgnitionByHall)
    angle_ignite = CalculateIgnition();
  else angle_ignite = 0.0f;

  if(isIgn && acis_config.params.isHallLearningMode)
    LearnIgnition();

  acis_hall_loop();

  acis_ignition_loop();

  if(isIgn)
  {
    if(acis_config.params.isForceTable)
    {
      table = acis_config.params.forceTableNumber;
    }
    else if(acis_config.params.isSwitchByExternal)
    {
      HAL_GPIO_WritePin(PROPANE_OUT_GPIO_Port, PROPANE_OUT_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PETROL_OUT_GPIO_Port, PETROL_OUT_Pin, GPIO_PIN_RESET);

      if(HAL_GPIO_ReadPin(PETROL_IN_GPIO_Port, PETROL_IN_Pin) == GPIO_PIN_RESET &&
          HAL_GPIO_ReadPin(PROPANE_IN_GPIO_Port, PROPANE_IN_Pin) == GPIO_PIN_SET)
        table = acis_config.params.switchPos1Table;
      else if(HAL_GPIO_ReadPin(PETROL_IN_GPIO_Port, PETROL_IN_Pin) == GPIO_PIN_SET &&
          HAL_GPIO_ReadPin(PROPANE_IN_GPIO_Port, PROPANE_IN_Pin) == GPIO_PIN_RESET)
        table = acis_config.params.switchPos2Table;
      else table = acis_config.params.switchPos0Table;
    }
    else
    {
      if(fuelsw == 0)
        table = acis_config.params.switchPos0Table;
      else if(fuelsw == 1)
        table = acis_config.params.switchPos1Table;
      else if(fuelsw == 2)
        table = acis_config.params.switchPos2Table;
    }

    if(table < TABLE_SETUPS_MAX)
    {
      table_current = table;
      valve_current = acis_config.tables[table].valve_channel;
      if(valve_current != valve_old)
      {
        valve_old = valve_current;
        rpm_last = now;
      }
      if(!acis_config.params.isSwitchByExternal)
      {
        if(acis_config.tables[table].valve_channel == ValveAllClosed ||
            (acis_config.tables[table].valve_timeout != 0 && DelayDiff(now, rpm_last) > acis_config.tables[table].valve_timeout))
        {
          HAL_GPIO_WritePin(PROPANE_OUT_GPIO_Port, PROPANE_OUT_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(PETROL_OUT_GPIO_Port, PETROL_OUT_Pin, GPIO_PIN_RESET);
        }
        else if(acis_config.tables[table].valve_channel == ValvePetrol)
        {
          HAL_GPIO_WritePin(PROPANE_OUT_GPIO_Port, PROPANE_OUT_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(PETROL_OUT_GPIO_Port, PETROL_OUT_Pin, GPIO_PIN_SET);
        }
        else if(acis_config.tables[table].valve_channel == ValvePropane)
        {
          HAL_GPIO_WritePin(PROPANE_OUT_GPIO_Port, PROPANE_OUT_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(PETROL_OUT_GPIO_Port, PETROL_OUT_Pin, GPIO_PIN_RESET);
        }
      }
    }
  }
  else
  {
    HAL_GPIO_WritePin(PROPANE_OUT_GPIO_Port, PROPANE_OUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PETROL_OUT_GPIO_Port, PETROL_OUT_Pin, GPIO_PIN_RESET);
  }

  if(!isIgn)
  {
    HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_RESET);
  }
  else if(!acis_config.params.isEconOutAsStrobe)
  {
    if(acis_config.params.isEconomEnabled)
    {
      if(isIdle && rpm > acis_config.params.EconRpmThreshold + 75)
        HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_RESET);
      else if(!isIdle || rpm < acis_config.params.EconRpmThreshold - 75)
        HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_SET);
    }
    else
      HAL_GPIO_WritePin(ECON_GPIO_Port, ECON_Pin, GPIO_PIN_SET);
  }
  TIME3 = Delay_Tick - TIME33;
}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  switch(msgBuf[0])
  {
    case PK_GeneralStatusRequestID :
      //PK_Copy(&PK_GeneralStatusRequest, msgBuf);
      PK_GeneralStatusResponse.Destination = xChaSrc;
      PK_GeneralStatusResponse.RealRPM = csps_getrpm();
      PK_GeneralStatusResponse.RPM = csps_getrpmgui();
      PK_GeneralStatusResponse.Pressure = map_getpressure();
      PK_GeneralStatusResponse.Load = map_getpressure() / 110000.0f * 100.0f;
      PK_GeneralStatusResponse.IgnitionAngle = angle_ignite;
      PK_GeneralStatusResponse.Voltage = power_voltage;
      PK_GeneralStatusResponse.valvenum = valve_current;
      PK_GeneralStatusResponse.check = csps_iserror() || (hall_error > 1.0f) || PK_GeneralStatusResponse.Load == 0.0f;
      PK_GeneralStatusResponse.tablenum = table_current;
      strcpy(PK_GeneralStatusResponse.tablename, PK_GeneralStatusResponse.tablenum < TABLE_SETUPS_MAX ? (char*)acis_config.tables[PK_GeneralStatusResponse.tablenum].name : (char*)"");
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


