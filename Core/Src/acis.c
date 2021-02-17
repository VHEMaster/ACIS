#include "acis.h"
#include "csps.h"
#include "map.h"
#include "crc.h"
#include "delay.h"
#include "packets.h"
#include "xCommand.h"
#include "xProFIFO.h"
#include "config.h"
#include <math.h>
#include <string.h>

sAcisConfig acis_config;

#define IGN_OVER_TIME 600000
#define IGN_SATURATION 1650
#define IGN_PULSE 2400
#define INITIAL_TIMEOUT 100000
static uint32_t ign14_sat = 0;
static uint32_t ign23_sat = 0;
static uint32_t ign14_time = 0;
static uint32_t ign23_time = 0;
static uint32_t ign14_prev = 0;
static uint32_t ign23_prev = 0;
static uint8_t ign_ftime = 1;

static volatile uint8_t CanDeinit = 0;
static volatile uint32_t hall_prev = 0;
static volatile float hall_angle = 0;
static volatile float hall_error = 0;
static volatile uint8_t hall_rotates = 0;
static volatile float angle_time = 0;
static volatile float angle_ignite = 0;
static volatile float angle_saturate = 0;
static uint8_t table_current = 0;
static uint8_t valve_current = 0;
static float engine_temperature = 0;
static float power_voltage = 0;
static int8_t switch_fuel_pos = 0;
static HAL_StatusTypeDef StatusInit = HAL_ERROR;

static sDragPoint DragPoints[DRAG_MAX_POINTS];
static uint32_t DragPointsCount = 0;
static float DragFromRPM = 0;
static float DragToRPM = 0;
static uint8_t DragReady = 0;
static uint8_t DragStarted = 0;
static uint8_t DragCompleted = 0;
static uint8_t DragStatus = 0;
static uint32_t DragTimeLast = 0;
static uint32_t DragStartTime = 0;

static volatile uint8_t savereq = 0;
static volatile uint8_t loadreq = 0;

static volatile uint8_t saturated_14 = 0;
static volatile uint8_t saturated_23 = 0;

static volatile eTransChannels savereqsrc;
static volatile eTransChannels loadreqsrc;

static uint16_t * volatile adc_buf_ptr = NULL;
static volatile uint32_t adc_buf_size = 0;
static volatile uint8_t adc_buf_ready = 0;

#define SENDING_BUFFER_SIZE (MAX_PACK_LEN)
static uint8_t buffSendingBuffer[SENDING_BUFFER_SIZE];

#define SENDING_QUEUE_SIZE (MAX_PACK_LEN*4)
static uint8_t buffSendingQueue[SENDING_QUEUE_SIZE];
static sProFIFO fifoSendingQueue;

extern TIM_HandleTypeDef htim4;

static int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length);

void acis_init(void)
{
  HAL_StatusTypeDef status = HAL_BUSY;
  protInit(&fifoSendingQueue, buffSendingQueue, 1, SENDING_QUEUE_SIZE);
  HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_SET);

  do
  {
    status = config_load(&acis_config);
  } while(status == HAL_BUSY);

  StatusInit = status;

  if(status != HAL_OK)
  {
    config_default(&acis_config);
    do
    {
      status = config_save(&acis_config);
    } while(status == HAL_BUSY);
  }

  if(status != HAL_OK)
  {
    config_default(&acis_config);
  }

  CanDeinit = 1;
  HAL_TIM_Base_Start_IT(&htim4);
}

static inline float gettempbyres(float resistance)
{
  const float koff = 0.1;
  const float koff_inv = 1.0f-koff;
  static float result_p = 0;
  const float resistances[22] = {100700,52700,28680,21450,16180,12300,9420,7280,5670,4450,3520,2796,2238,1802,1459,1188,973,667,467,332,241,177};
  const float temperatures[22] = {-40,-30,-20,-15,-10,-4,0,5,10,15,20,25,30,35,40,45,50,60,70,80,90,100};
  float result = 0.0f;
  uint8_t index1, index2;
  float temp1 = 0.0f, temp2 = 0.0f, mult, tempt1, tempt2;

  if(resistance >= resistances[0])
  {
    index1 = 0;
    index2 = 1;
    temp1 = resistances[index1];
    temp2 = resistances[index2];
  }
  else if(resistance <= resistances[(sizeof(resistances) / sizeof(float)) - 1])
  {
    index1 = (sizeof(resistances) / sizeof(float)) - 2;
    index2 = (sizeof(resistances) / sizeof(float)) - 1;
    temp1 = resistances[index1];
    temp2 = resistances[index2];
  }
  else
  {
    for(int i = 1; i < (sizeof(resistances) / sizeof(float)); i++)
    {
      temp1 = resistances[i-1];
      temp2 = resistances[i];
      if(temp1 > resistance && temp2 < resistance)
      {
        index1 = i-1;
        index2 = i;
        break;
      }
      temp1 = 0.0f;
      temp2 = 0.0f;
    }
  }

  if(temp1 != 0.0f || temp2 != 0.0f)
  {
    tempt1 = temperatures[index1];
    tempt2 = temperatures[index2];
    if(temp2 != temp1)
    {
      mult = (resistance - temp1) / (temp2 - temp1);
      result = (tempt2 - tempt1) * mult + tempt1;
    }
    else result = (tempt1 + tempt2) / 2.0f;
  }
  if(result > 150.0f) result = 150.0f;
  result_p = result * koff + result_p * koff_inv;
  return result_p;
}

void acis_deinitIfNeed(void)
{
  if(CanDeinit && HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_SET)
    HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_RESET);
  else HAL_GPIO_WritePin(VDD3V3EN_GPIO_Port, VDD3V3EN_Pin, GPIO_PIN_SET);
}

void acis_adc_irq(uint16_t * data, uint32_t size)
{
  adc_buf_ptr = data;
  adc_buf_size = size;
  adc_buf_ready = 1;
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

static inline void acis_saturate(int8_t index)
{
  static uint8_t cylinder14 = 0;
  static uint8_t cylinder23 = 0;
  static int8_t cutoffcnt0 = 0;
  static int8_t cutoffcnt1 = 0;
  static int8_t cutoffcnt2 = 0;
  static int8_t cutoffcnt3 = 0;
  static int8_t cutoffcnt4 = 0;
  static int8_t cutoffcnt5 = 0;
  static int8_t cutoffcnt6 = 0;
  static int8_t index_prev = -1;

  if(acis_config.params.isCutoffEnabled)
  {
    float rpm = csps_getrpmgui();
    int32_t mode = acis_config.params.CutoffMode;
    float cutoffrpm = acis_config.params.CutoffRPM;
    if(rpm >= cutoffrpm)
    {
      if(mode == 0)
        return;
      else if(mode == 1)
      {
        if(cutoffcnt0 == -1)
          cutoffcnt0 = 0;
      }
      else if(mode == 2)
      {
        if(cutoffcnt1 == -1)
          cutoffcnt1 = 0;
      }
      else if(mode == 3)
      {
        if(cutoffcnt2 == -1)
          cutoffcnt2 = 0;
      }
      else if(mode == 4)
      {
        if(cutoffcnt3 == -1)
          cutoffcnt3 = 0;
      }
      else if(mode == 5)
      {
        if(cutoffcnt4 == -1)
          cutoffcnt4 = 0;
      }
      else if(mode == 6)
      {
        if(cutoffcnt5 == -1)
          cutoffcnt5 = 0;
      }
      else if(mode == 7)
      {
        if(cutoffcnt6 == -1)
          cutoffcnt6 = 0;
      }
    }
  }

  if(index_prev != index)
  {
    index_prev = index;

    if(cutoffcnt0 >= 0)
    {
      if(++cutoffcnt0 > 36)
        cutoffcnt0 = -1;
      else if(cutoffcnt0 <= 36-4)
        return;
    }

    if(cutoffcnt1 >= 0)
    {
      if(++cutoffcnt1 > 24)
        cutoffcnt1 = -1;
      else if(cutoffcnt1 <= 24-4)
        return;
    }

    if(cutoffcnt2 >= 0)
    {
      if(++cutoffcnt2 > 16)
        cutoffcnt2 = -1;
      else if(cutoffcnt2 <= 16-4)
        return;
    }

    if(cutoffcnt3 >= 0)
    {
      if(++cutoffcnt3 > 8)
        cutoffcnt3 = -1;
      else if(cutoffcnt3 <= 8-4)
        return;
    }

    if(cutoffcnt4 >= 0)
    {
      if(++cutoffcnt4 > 20)
        cutoffcnt4 = -1;
      else if((cutoffcnt4 % 5) != 0)
        return;
    }

    if(cutoffcnt5 >= 0)
    {
      if(++cutoffcnt5 > 12)
        cutoffcnt5 = -1;
      else if((cutoffcnt5 % 3) != 0)
        return;
    }

    if(cutoffcnt6 >= 0)
    {
      if(++cutoffcnt6 > 12)
        cutoffcnt6 = -1;
      else if((cutoffcnt6 % 3) == 0)
        return;
    }
  }

  if(index == 0)
  {
    acis_saturate_14();
    cylinder14 ^= 1;
  }
  else if(index == 1)
  {
    acis_saturate_23();
    cylinder23 ^= 1;
  }

}

static inline void acis_ignition_loop(void)
{
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
    acis_ignite(hall_cylinders - 1);

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
  float rpm = csps_getrpmgui();
  float pressure = map_getpressure();
  float temperature = engine_temperature;

  if(map_iserror())
    pressure = 100000.0f;

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
      if(isIdle && table->idles_count > 0)
      {
        if(table->idles_count == 1)
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
            if(temprpm1 != temprpm2)
            {
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

          tempign11 = table->ignitions[pressindex1][rpmindex1];
          tempign12 = table->ignitions[pressindex1][rpmindex2];
          tempign21 = table->ignitions[pressindex2][rpmindex1];
          tempign22 = table->ignitions[pressindex2][rpmindex2];
          if((temprpm1 != 0.0f || temprpm2 != 0.0f) && (temppress1 != 0.0f || temppress2 != 0.0f))
          {

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

      if(acis_config.params.isTemperatureEnabled && table->temperatures_count > 0)
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
            tempign1 = table->temperature_ignitions[tempindex1];
            tempign2 = table->temperature_ignitions[tempindex2];
            if(temptemp1 != temptemp2)
            {
              mult = (temperature - temptemp1) / (temptemp2 - temptemp1);
              angle += (tempign2 - tempign1) * mult + tempign1;
            }
            else angle += (tempign1 + tempign2) / 2.0f;
          }
        }
      }
    }
    angle += table->octane_corrector;
    if(angle > 90.0f) angle = 90.0f;
    if(angle < -60.0f) angle = -60.0f;
  }
  return angle;
}

static void LearnIgnition(void)
{
  if(!hall_rotates || csps_iserror() || map_iserror())
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
      if(isIdle && table->idles_count > 0)
      {
        if(table->idles_count == 1)
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
  static float ignite = 0;
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

  const float angle_koff = 0.001f;
  const float angle_koff_inv = 1.0f - angle_koff;

  if(!isIgn)
    return;

  if(acis_config.params.isEconOutAsStrobe)
  {
    if(csps_isrotates() && angle[0] > -0.2f && angle[0] < 1.5f)
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

  float found = csps_isfound();


  if(found)
  {
    if(period < IGN_SATURATION + IGN_PULSE)
    {
      time_sat = period * ((float)IGN_SATURATION / (float)(IGN_SATURATION + IGN_PULSE));
    }

    float saturate = time_sat / uspa;

    ignite = angle_ignite * angle_koff + angle_ignite * angle_koff_inv;
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
        if(start_ign_state[i] && DelayDiff(now, start_ign_last[i]) > IGN_SATURATION)
        {
          acis_ignite(i);
          start_ign_state[i] = 0;
          start_ign_last[i] = now;
        }
        else if(!start_ign_state[i] && DelayDiff(now, start_ign_last[i]) > IGN_PULSE/2)
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
}

inline void acis_loop(void)
{
  HAL_StatusTypeDef flashstatus = HAL_BUSY;
  static uint8_t issaving = 0;
  static uint8_t isloading = 0;
  static uint8_t sending = 0;
  static uint8_t destination = 0;
  static uint8_t size = 0;
  static uint32_t rpm_last = 0;
  static eValveChannel valve_old = ValveAllClosed;
  uint32_t now = Delay_Tick;
  float rpm = csps_getrpm();
  float pressure = map_getpressure();
  float load = map_getpressure() / 110000.0f * 100.0f;
  float temp;
  float meter_resistance;
  uint32_t voltage_raw = 0;
  uint32_t temperature_raw = 0;
  uint8_t isIdle = HAL_GPIO_ReadPin(SENS_ACC_GPIO_Port,SENS_ACC_Pin) == GPIO_PIN_SET;
  uint8_t isIgn = HAL_GPIO_ReadPin(MCU_IGN_GPIO_Port, MCU_IGN_Pin) == GPIO_PIN_RESET;
  uint8_t * pnt;
  int8_t status;
  uint8_t table = 0;
  uint8_t fuelsw = switch_fuel_pos;
  uint16_t * adc_buf;
  uint32_t adc_size;

  if(rpm_last == 0 || rpm > 10.0f || csps_isrotates()) rpm_last = now;

  /* =========================== Packets Sending =========================== */
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

  /* =========================== Voltage & Temperature ADC =========================== */
  if(adc_buf_ready)
  {
    adc_buf = adc_buf_ptr;
    adc_size = adc_buf_size;
    adc_buf_ready = 0;
    if(adc_buf)
    {
      for(int i = 0; i < adc_size;)
      {
        temperature_raw += adc_buf[i++];
        voltage_raw += adc_buf[i++];
      }
      temperature_raw /= adc_size >> 1;
      voltage_raw /= adc_size >> 1;

      power_voltage = voltage_raw / 65535.0f * 3.34f * 5.5f;
      temp = temperature_raw / 65535.0f * 3.34f * 11.0f;

      if(temp < 1.0f)
        engine_temperature = 150.0f;
      else
      {
        if(temp >= power_voltage)
          temp = power_voltage;
        meter_resistance = 200.0f;
        meter_resistance = (meter_resistance / (1.0f - (temp/power_voltage))) - meter_resistance;
        temp = gettempbyres(meter_resistance);
        if(temp < -40.0f)
          engine_temperature = -40.0f;
        else if(temp > 150.0f)
          engine_temperature = 150.0f;
        else
          engine_temperature = temp;
      }
    }
  }

  /* =========================== Save & Load Procedures =========================== */
  if(!issaving && !isloading)
  {
    if(savereq && !isloading)
      issaving = 1, CanDeinit = 0;
    else if(loadreq && !issaving)
      isloading = 1;
  }

  if(issaving)
  {
    CanDeinit = 0;
    flashstatus = config_save(&acis_config);
    if(flashstatus != HAL_BUSY)
    {
      PK_SaveConfigAcknowledge.Destination = savereqsrc;
      PK_SaveConfigAcknowledge.ErrorCode = flashstatus == HAL_OK ? 0 : 1;
      protPushSequence(&fifoSendingQueue, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      StatusInit = flashstatus;
      issaving = 0;
      CanDeinit = 1;
      savereq = 0;
    }
  }
  else if(isloading)
  {
    flashstatus = config_load(&acis_config);
    if(flashstatus != HAL_BUSY)
    {
      PK_RestoreConfigAcknowledge.Destination = loadreqsrc;
      PK_RestoreConfigAcknowledge.ErrorCode = flashstatus == HAL_OK ? 0 : 1;
      protPushSequence(&fifoSendingQueue, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      StatusInit = flashstatus;
      isloading = 0;
      loadreq = 0;
    }
  }

  /* =========================== Ignition Handling =========================== */
  if(isIgn && !acis_config.params.isIgnitionByHall)
    angle_ignite = CalculateIgnition();
  else angle_ignite = 0.0f;

  if(isIgn && acis_config.params.isHallLearningMode)
    LearnIgnition();

  acis_hall_loop();

  acis_ignition_loop();


  /* =========================== Table and Valves handling =========================== */
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


  /* =========================== Economizer Handling =========================== */
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

  /* =========================== DRAG =========================== */
  if(DragReady)
  {
    if(DragCompleted)
    {
      DragReady = 0;
      DragStarted = 0;
    }
    else
    {
      if(DragStarted)
      {
        if(DelayDiff(now, DragTimeLast) >= DRAG_POINTS_DISTANCE)
        {
          DragTimeLast = now;
          if(DragPointsCount < DRAG_MAX_POINTS)
          {
            DragPoints[DragPointsCount].Ignition = angle_ignite;
            DragPoints[DragPointsCount].RPM = csps_getrpmgui();
            DragPoints[DragPointsCount].Pressure = pressure;
            DragPoints[DragPointsCount].Load = load;
            DragPoints[DragPointsCount].Time = DelayDiff(now, DragStartTime);
            DragPointsCount++;

            if(DragFromRPM < DragToRPM)
            {
              if(rpm >= DragToRPM)
              {
                DragStarted = 0;
                DragStatus = 0;
                DragReady = 0;
                DragCompleted = 1;
                DragTimeLast = 0;
              }
              else if(rpm < DragFromRPM - 100.0f)
              {
                DragStarted = 0;
                DragStatus = 4;
                DragReady = 0;
                DragCompleted = 1;
                DragTimeLast = 0;
              }
            }
            else if(DragFromRPM > DragToRPM)
            {
              if(rpm <= DragToRPM)
              {
                DragStarted = 0;
                DragStatus = 0;
                DragReady = 0;
                DragCompleted = 1;
                DragTimeLast = 0;
              }
              else if(rpm > DragFromRPM + 100.0f)
              {
                DragStarted = 0;
                DragStatus = 4;
                DragReady = 0;
                DragCompleted = 1;
                DragTimeLast = 0;
              }
            }
          }
          else
          {
            DragStarted = 0;
            DragStatus = 5;
            DragReady = 0;
            DragCompleted = 1;
            DragTimeLast = 0;
          }
        }
      }
      else
      {
        if(DragFromRPM < DragToRPM)
        {
          if(rpm > DragFromRPM)
          {
            if(DragTimeLast != 0)
            {
              DragTimeLast = now;
              DragStarted = 1;
              DragStatus = 0;
              DragPointsCount = 0;
              DragStartTime = now;
              DragPoints[DragPointsCount].Ignition = angle_ignite;
              DragPoints[DragPointsCount].RPM = rpm;
              DragPoints[DragPointsCount].Pressure = pressure;
              DragPoints[DragPointsCount].Load = load;
              DragPoints[DragPointsCount].Time = DelayDiff(now, DragStartTime);
              DragPointsCount++;
            }
          }
          else
          {
            DragTimeLast = now;
          }
        }
        else if(DragFromRPM > DragToRPM)
        {
          if(rpm < DragFromRPM)
          {
            if(DragTimeLast != 0)
            {
              DragTimeLast = now;
              DragStarted = 1;
              DragPointsCount = 0;
              DragStartTime = now;
              DragPoints[DragPointsCount].Ignition = angle_ignite;
              DragPoints[DragPointsCount].RPM = rpm;
              DragPoints[DragPointsCount].Pressure = pressure;
              DragPoints[DragPointsCount].Load = load;
              DragPoints[DragPointsCount].Time = DelayDiff(now, DragStartTime);
              DragPointsCount++;
            }
          }
          else
          {
            DragTimeLast = now;
          }
        }
      }
    }
  }


}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  static uint32_t pclastsent = 0;
  uint32_t now = Delay_Tick;
  uint32_t offset;
  uint32_t size;
  uint32_t table;
  uint32_t tablesize;
  uint32_t configsize;
  uint32_t dragpoint;
  uint32_t realconfigsize = (uint32_t)&acis_config.tables[0] - (uint32_t)&acis_config;
  if(xChaSrc == etrPC)
  {
    if(DelayDiff(now, pclastsent) > 500000)
    {
      pclastsent = now;
      PK_PcConnected.Destination = etrCTRL;
      protPushSequence(&fifoSendingQueue, &PK_PcConnected, sizeof(PK_PcConnected));
    }
  }
  switch(msgBuf[0])
  {
    case PK_PingID :
      PK_Copy(&PK_Ping, msgBuf);
      PK_Pong.RandomPong = PK_Ping.RandomPing;
      PK_Pong.Destination = xChaSrc;
      protPushSequence(&fifoSendingQueue, &PK_Pong, sizeof(PK_Pong));
      break;

    case PK_PongID :
      PK_Copy(&PK_Pong, msgBuf);
      (void)PK_Pong.RandomPong;
      break;

    case PK_GeneralStatusRequestID :
      //PK_Copy(&PK_GeneralStatusRequest, msgBuf);
      PK_GeneralStatusResponse.Destination = xChaSrc;
      PK_GeneralStatusResponse.RealRPM = csps_getrpm();
      PK_GeneralStatusResponse.RPM = csps_getrpmgui();
      PK_GeneralStatusResponse.Pressure = map_getpressure();
      PK_GeneralStatusResponse.Load = map_getpressure() / 110000.0f * 100.0f;
      PK_GeneralStatusResponse.IgnitionAngle = angle_ignite;
      PK_GeneralStatusResponse.Voltage = power_voltage;
      PK_GeneralStatusResponse.Temperature = engine_temperature;
      PK_GeneralStatusResponse.valvenum = valve_current;
      PK_GeneralStatusResponse.check = (PK_GeneralStatusResponse.RPM < 10.0f && PK_GeneralStatusResponse.Pressure < 85000) ||
          csps_iserror() || (hall_error > 1.0f) || PK_GeneralStatusResponse.Load == 0.0f || StatusInit != HAL_OK;
      PK_GeneralStatusResponse.tablenum = table_current;
      if(StatusInit == HAL_OK) strcpy(PK_GeneralStatusResponse.tablename, PK_GeneralStatusResponse.tablenum < TABLE_SETUPS_MAX ? (char*)acis_config.tables[PK_GeneralStatusResponse.tablenum].name : (char*)"");
      else strcpy(PK_GeneralStatusResponse.tablename, "Flash Error");
      protPushSequence(&fifoSendingQueue, &PK_GeneralStatusResponse, sizeof(PK_GeneralStatusResponse));
      break;

    case PK_TableMemoryRequestID :
      PK_Copy(&PK_TableMemoryRequest, msgBuf);
      PK_TableMemoryData.Destination = xChaSrc;
      offset = PK_TableMemoryData.offset = PK_TableMemoryRequest.offset;
      size = PK_TableMemoryData.size = PK_TableMemoryRequest.size;
      table = PK_TableMemoryData.table = PK_TableMemoryRequest.table;
      tablesize = PK_TableMemoryData.tablesize = PK_TableMemoryRequest.tablesize;
      PK_TableMemoryData.ErrorCode = 0;

      if(tablesize != sizeof(sAcisIgnTable))
        PK_TableMemoryData.ErrorCode = 1;

      if(size + offset > sizeof(sAcisIgnTable))
        PK_TableMemoryData.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sAcisIgnTable))
        PK_TableMemoryData.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryData.ErrorCode = 4;

      if(PK_TableMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&acis_config.tables[table])[offset], size);
        memset(&PK_TableMemoryData.data[size], 0, sizeof(PK_TableMemoryData.data) - size);
        PK_TableMemoryData.crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));
      }
      else
      {
        memset(&PK_TableMemoryData.data[0], 0, sizeof(PK_TableMemoryData.data));
        PK_TableMemoryData.crc = 0;
      }
      protPushSequence(&fifoSendingQueue, &PK_TableMemoryData, sizeof(PK_TableMemoryData));
      break;

    case PK_TableMemoryDataID :
      PK_Copy(&PK_TableMemoryData, msgBuf);
      PK_TableMemoryAcknowledge.Destination = xChaSrc;
      offset = PK_TableMemoryAcknowledge.offset = PK_TableMemoryData.offset;
      size = PK_TableMemoryAcknowledge.size = PK_TableMemoryData.size;
      table = PK_TableMemoryAcknowledge.table = PK_TableMemoryData.table;
      tablesize = PK_TableMemoryAcknowledge.tablesize = PK_TableMemoryData.tablesize;
      PK_TableMemoryAcknowledge.ErrorCode = 0;

      if(tablesize != sizeof(sAcisIgnTable))
        PK_TableMemoryAcknowledge.ErrorCode = 1;

      if(size + offset > sizeof(sAcisIgnTable))
        PK_TableMemoryAcknowledge.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sAcisIgnTable))
        PK_TableMemoryAcknowledge.ErrorCode = 3;

      if(table >= TABLE_SETUPS_MAX)
        PK_TableMemoryAcknowledge.ErrorCode = 4;

      if(PK_TableMemoryAcknowledge.ErrorCode == 0)
      {
        uint16_t crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));
        if(crc == PK_TableMemoryData.crc)
        {
          memcpy(&((uint8_t*)&acis_config.tables[table])[offset], &PK_TableMemoryData.data[0], size);
        }
        else
          PK_TableMemoryAcknowledge.ErrorCode = 5;
      }

      protPushSequence(&fifoSendingQueue, &PK_TableMemoryAcknowledge, sizeof(PK_TableMemoryAcknowledge));
      break;

    case PK_ConfigMemoryRequestID :
      PK_Copy(&PK_ConfigMemoryRequest, msgBuf);
      PK_ConfigMemoryData.Destination = xChaSrc;
      offset = PK_ConfigMemoryData.offset = PK_ConfigMemoryRequest.offset;
      size = PK_ConfigMemoryData.size = PK_ConfigMemoryRequest.size;
      configsize = PK_ConfigMemoryData.configsize = PK_ConfigMemoryRequest.configsize;
      PK_ConfigMemoryData.ErrorCode = 0;

      if(configsize != realconfigsize)
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > realconfigsize)
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > realconfigsize || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&acis_config)[offset], size);
        memset(&PK_ConfigMemoryData.data[size], 0, sizeof(PK_ConfigMemoryData.data) - size);
        PK_ConfigMemoryData.crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));
      }
      else
      {
        memset(&PK_ConfigMemoryData.data[0], 0, sizeof(PK_ConfigMemoryData.data));
        PK_ConfigMemoryData.crc = 0;
      }
      protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryData, sizeof(PK_ConfigMemoryData));
      break;

    case PK_ConfigMemoryDataID :
      PK_Copy(&PK_ConfigMemoryData, msgBuf);
      PK_ConfigMemoryAcknowledge.Destination = xChaSrc;
      offset = PK_ConfigMemoryAcknowledge.offset = PK_ConfigMemoryData.offset;
      size = PK_ConfigMemoryAcknowledge.size = PK_ConfigMemoryData.size;
      configsize = PK_ConfigMemoryAcknowledge.configsize = PK_ConfigMemoryData.configsize;
      PK_ConfigMemoryAcknowledge.ErrorCode = 0;

      if(configsize != realconfigsize)
        PK_ConfigMemoryData.ErrorCode = 1;

      if(size + offset > realconfigsize)
        PK_ConfigMemoryData.ErrorCode = 2;

      if(size > realconfigsize || size > PACKET_CONFIG_MAX_SIZE)
        PK_ConfigMemoryData.ErrorCode = 3;

      if(PK_ConfigMemoryAcknowledge.ErrorCode == 0)
      {
        uint16_t crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));
        if(crc == PK_ConfigMemoryData.crc)
        {
          memcpy(&((uint8_t*)&acis_config)[offset], &PK_ConfigMemoryData.data[0], size);
        }
        else
          PK_ConfigMemoryAcknowledge.ErrorCode = 5;
      }

      protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryAcknowledge, sizeof(PK_ConfigMemoryAcknowledge));
      break;

    case PK_SaveConfigID :
      if(!savereq)
      {
        savereqsrc = xChaSrc;
        savereq = 1;
        CanDeinit = 0;
      }
      else if(loadreq)
      {
        PK_SaveConfigAcknowledge.Destination = xChaSrc;
        PK_SaveConfigAcknowledge.ErrorCode = 3;
        protPushSequence(&fifoSendingQueue, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      else
      {
        PK_SaveConfigAcknowledge.Destination = xChaSrc;
        PK_SaveConfigAcknowledge.ErrorCode = 2;
        protPushSequence(&fifoSendingQueue, &PK_SaveConfigAcknowledge, sizeof(PK_SaveConfigAcknowledge));
      }
      break;


    case PK_RestoreConfigID :
      if(!loadreq)
      {
        loadreqsrc = xChaSrc;
        loadreq = 1;
      }
      else if(savereq)
      {
        PK_RestoreConfigAcknowledge.Destination = xChaSrc;
        PK_RestoreConfigAcknowledge.ErrorCode = 3;
        protPushSequence(&fifoSendingQueue, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      else
      {
        PK_RestoreConfigAcknowledge.Destination = xChaSrc;
        PK_RestoreConfigAcknowledge.ErrorCode = 2;
        protPushSequence(&fifoSendingQueue, &PK_RestoreConfigAcknowledge, sizeof(PK_RestoreConfigAcknowledge));
      }
      break;

    case PK_DragStartID :
      PK_Copy(&PK_DragStart, msgBuf);
      PK_DragStartAcknowledge.Destination = xChaSrc;
      DragFromRPM = PK_DragStartAcknowledge.FromRPM = PK_DragStart.FromRPM;
      DragToRPM = PK_DragStartAcknowledge.ToRPM = PK_DragStart.ToRPM;
      DragStarted = 0;
      DragReady = 0;
      DragCompleted = 0;
      DragPointsCount = 0;
      DragStartTime = 0;
      DragTimeLast = 0;
      if(DragFromRPM > 100.0f && DragFromRPM < 20000.0f && DragToRPM > 100.0f && DragToRPM < 20000.0f && fabsf(DragFromRPM - DragToRPM) > 70.0f)
      {
        DragStatus = PK_DragStartAcknowledge.ErrorCode = 0;
        DragReady = 1;
      }
      else DragStatus = PK_DragStartAcknowledge.ErrorCode = 1;
      protPushSequence(&fifoSendingQueue, &PK_DragStartAcknowledge, sizeof(PK_DragStartAcknowledge));
      break;

    case PK_DragStopID :
      PK_Copy(&PK_DragStop, msgBuf);
      PK_DragStopAcknowledge.Destination = xChaSrc;
      PK_DragStopAcknowledge.FromRPM = PK_DragStop.FromRPM;
      PK_DragStopAcknowledge.ToRPM = PK_DragStop.ToRPM;
      DragStatus = PK_DragStopAcknowledge.ErrorCode = 0;
      DragStarted = 0;
      DragReady = 0;
      DragCompleted = 0;
      DragPointsCount = 0;
      DragStartTime = 0;
      if(DragFromRPM != PK_DragStop.FromRPM || DragToRPM != PK_DragStop.ToRPM)
      {
        PK_DragStopAcknowledge.ErrorCode = 2;
      }
      protPushSequence(&fifoSendingQueue, &PK_DragStopAcknowledge, sizeof(PK_DragStopAcknowledge));
      break;

    case PK_DragUpdateRequestID :
      PK_Copy(&PK_DragUpdateRequest, msgBuf);
      PK_DragUpdateResponse.Destination = xChaSrc;
      PK_DragUpdateResponse.ErrorCode = 0;
      PK_DragUpdateResponse.FromRPM = PK_DragUpdateRequest.FromRPM;
      PK_DragUpdateResponse.ToRPM = PK_DragUpdateRequest.ToRPM;
      PK_DragUpdateResponse.CurrentRPM = csps_getrpm();
      PK_DragUpdateResponse.CurrentIgnition = angle_ignite;
      PK_DragUpdateResponse.CurrentPressure = map_getpressure();
      PK_DragUpdateResponse.CurrentLoad = map_getpressure() / 110000.0f * 100.0f;
      PK_DragUpdateResponse.TotalPoints = DragPointsCount;
      PK_DragUpdateResponse.Started = DragStarted;
      PK_DragUpdateResponse.Completed = DragCompleted;
      PK_DragUpdateResponse.Time = DragStartTime > 0 ? DelayDiff(now, DragStartTime) : 0;
      if(DragFromRPM != PK_DragUpdateRequest.FromRPM || DragToRPM != PK_DragUpdateRequest.ToRPM)
      {
        PK_DragUpdateResponse.ErrorCode = 2;
      }
      else if(DragStatus > 0) PK_DragUpdateResponse.ErrorCode = DragStatus + 10;
      protPushSequence(&fifoSendingQueue, &PK_DragUpdateResponse, sizeof(PK_DragUpdateResponse));
      break;

    case PK_DragPointRequestID :
      PK_Copy(&PK_DragPointRequest, msgBuf);
      PK_DragPointResponse.Destination = xChaSrc;
      PK_DragPointResponse.FromRPM = PK_DragStop.FromRPM;
      PK_DragPointResponse.ToRPM = PK_DragStop.ToRPM;
      PK_DragPointResponse.ErrorCode = 0;
      dragpoint = PK_DragPointResponse.PointNumber = PK_DragPointRequest.PointNumber;
      if(dragpoint >= DragPointsCount)
      {
        PK_DragPointResponse.Point.Pressure = 0;
        PK_DragPointResponse.Point.RPM = 0;
        PK_DragPointResponse.Point.Load = 0;
        PK_DragPointResponse.Point.Ignition = 0;
        PK_DragPointResponse.Point.Time = 0;
        PK_DragPointResponse.ErrorCode = 3;
      }
      else
      {
        PK_DragPointResponse.Point = DragPoints[dragpoint];
        if(DragFromRPM != PK_DragPointRequest.FromRPM || DragToRPM != PK_DragPointRequest.ToRPM)
        {
          PK_DragPointResponse.ErrorCode = 2;
        }
      }

      protPushSequence(&fifoSendingQueue, &PK_DragPointResponse, sizeof(PK_DragPointResponse));
      break;

    case PK_ConfigMemoryAcknowledgeID :
      PK_Copy(&PK_ConfigMemoryAcknowledge, msgBuf);
      if(PK_ConfigMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_TableMemoryAcknowledgeID :
      PK_Copy(&PK_TableMemoryAcknowledge, msgBuf);
      if(PK_TableMemoryAcknowledge.ErrorCode != 0)
      {

      }
      break;

    case PK_FuelSwitchID :
      PK_Copy(&PK_FuelSwitch, msgBuf);
      if(PK_FuelSwitch.FuelSwitchPos < 3)
        switch_fuel_pos = PK_FuelSwitch.FuelSwitchPos;
      break;

    default:
      break;
  }
}

inline int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length)
{
  return xSender(xChaDst, (uint8_t*)msgBuf, length);
}


