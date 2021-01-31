#include "csps.h"
#include "delay.h"
#include <math.h>
#include <string.h>

#define ANGLE_INITIAL -114.0f
#define IRQ_SIZE 8
#define DATA_SIZE 16

typedef struct
{
  uint32_t DelayCur;
  uint32_t DelayPrev;
  float AngleCur14;
  float AnglePrev14;
  float AngleCur23;
  float AnglePrev23;
  float RPM;
  float uSPA;
}sCspsData;


uint32_t cspc_irq_data[IRQ_SIZE] = {0};
volatile uint8_t csps_found = 0;
volatile uint8_t csps_rotates = 0;
volatile uint32_t csps_pulse_last = 0;
volatile uint32_t csps_last = 0;
volatile float csps_errors = 0;
volatile float csps_rpm = 0;
volatile float csps_uspa = 0;
volatile float csps_period = 0;

static sCspsData CspsData[DATA_SIZE];
static sCspsData * volatile CspsDataPtr = &CspsData[0];

void csps_init(void)
{

}

inline void csps_exti(void)
{
  static float csps_angle14 = 0, csps_angle23 = 0;
  static float cs14_p = 0, cs23_p = 0;
  static float average_prev = 0;
  static uint8_t dataindex = 0;
  static uint32_t t1 = 0;
  static uint32_t t2 = 0;
  uint32_t i, ticks, cur, prev;
  sCspsData data;
  float cs14, cs23;
  float average = 0;
  float diff = 0;
  float rpm_koff = 1.0f / 60.0f;
  float uspa_koff = 1.0f / 10.0f;

  cur = Delay_Tick;
  csps_pulse_last = cur;
  for(i = 1; i < IRQ_SIZE; i++)
    cspc_irq_data[i - 1] = cspc_irq_data[i];
  cspc_irq_data[IRQ_SIZE - 1] = cur;
  if(cspc_irq_data[0] == 0)
    return;
  prev = cspc_irq_data[IRQ_SIZE - 2];
  csps_rotates = 1;

  t1++;

  for(i = 1; i < IRQ_SIZE; i++)
  {
    average += DelayDiff(cspc_irq_data[i], cspc_irq_data[i - 1]);;
  }
  average /= (float)(IRQ_SIZE - 1);

  if(average / average_prev > 1.0f + 1.0f / IRQ_SIZE)
  {
    if(++t2 == 2)
    {
      ticks = t1;
      t1 = 0;
      t2 = 0;
      if(ticks != 116)
        csps_errors += 1.0f;
      else
      {
        csps_last = cur;
        csps_found = 1;
      }
    }
  }
  else if(t1 >= 116)
  {
    t1 = 1;
    csps_errors += 1.0f;
  }

  average_prev = average;

  if(csps_found)
  {
    switch(t1)
    {
      case 0:
        csps_angle14 = ANGLE_INITIAL;
        if (ANGLE_INITIAL > 0)
          csps_angle23 = ANGLE_INITIAL - 180.0f;
        else
          csps_angle23 = ANGLE_INITIAL + 180.0f;
        cs14_p = csps_angle14 - 3.0f;
        cs23_p = csps_angle23 - 3.0f;
        prev = DelayDiff(cur, (DelayDiff(cur, prev) / 3));
        break;
      case 1:
        prev = DelayDiff(cur, (DelayDiff(cur, prev) / 3));
        /* no break */
      default:
        cs14 = csps_angle14 + 3.0f;
        if(cs14 > 180.0f) csps_angle14 = cs14 - 360.0f;
        else csps_angle14 = cs14;

        cs23 = csps_angle23 + 3.0f;
        if(cs23 > 180.0f) csps_angle23 = cs23 - 360.0f;
        else csps_angle23 = cs23;

        cs14_p += 3.0f;
        if(cs14_p > 180.0f)
          cs14_p -= 360.0f;

        cs23_p += 3.0f;
        if(cs23_p > 180.0f)
          cs23_p -= 360.0f;
        break;
    }

    if(csps_rpm == 0.0f)
      rpm_koff = 1.0f;

    diff = (float)DelayDiff(cur, prev);

    if(csps_period > 1000000.0f)
      csps_period = 1000000.0f;

    csps_period = csps_period * (1.0f - rpm_koff) + (diff * 120.0f) * rpm_koff;
    csps_rpm = 1000000.0f / csps_period * 60.0f;
    csps_uspa = csps_uspa * (1.0f - uspa_koff) + (diff / 3.0f) * uspa_koff;

    data.AngleCur14 = csps_angle14;
    data.AngleCur23 = csps_angle23;
    data.AnglePrev14 = cs14_p;
    data.AnglePrev23 = cs23_p;
    data.DelayPrev = prev;
    data.DelayCur = cur;
    data.RPM = csps_rpm;
    data.uSPA = csps_uspa;

  }
  else
  {
    data.AngleCur14 = 0;
    data.AngleCur23 = 0;
    data.AnglePrev14 = 0;
    data.AnglePrev23 = 0;
    data.DelayPrev = 0;
    data.DelayCur = 0;
    data.RPM = 0;
    csps_rpm = 0;
    data.uSPA = 1.0f / csps_rpm;
    csps_period = 1.0f / csps_rpm;
  }
  CspsData[dataindex] = data;
  CspsDataPtr = &CspsData[dataindex];
  if(++dataindex >= DATA_SIZE)
    dataindex = 0;

}

inline float csps_correctangle(float angle)
{
  if(angle > 180.0f)
    angle -= 360.0f;
  else if(angle <= -180.0f)
    angle += 360.0f;
  return angle;
}

/*
inline float csps_anglediff(float a, float b)
{
  if(a >= b)
    return a - b;
  else return 360 + a - b;
}
*/
volatile float rettttt;
inline float csps_getangle14(void)
{
  static float angle_prev = 0;
  float now = Delay_Tick;
  float angle, acur, aprev, mult, cur;
  sCspsData data = *CspsDataPtr;

  cur = DelayDiff(data.DelayCur, data.DelayPrev);
  now = DelayDiff(now, data.DelayPrev);

  acur = data.AngleCur14;
  aprev = data.AnglePrev14;

  if(acur < aprev)
    acur += 360.0f;

  angle = acur - aprev;
  mult = angle / cur;
  angle = mult * now + aprev;

  while(angle > 180.0f)
    angle -= 360.0f;

  if((angle - angle_prev < 0.0f && angle - angle_prev > -90.0f) || angle - angle_prev > 90.0f)
  {
    angle = angle_prev;
  }
  angle_prev = angle;

  return angle;
}

inline float csps_getangle23(void)
{
  static float angle_prev = 0;
  float now = Delay_Tick;
  float angle, acur, aprev, mult, cur;
  sCspsData data = *CspsDataPtr;

  cur = DelayDiff(data.DelayCur, data.DelayPrev);
  now = DelayDiff(now, data.DelayPrev);

  acur = data.AngleCur23;
  aprev = data.AnglePrev23;

  if(acur < aprev)
    acur += 360.0f;

  angle = acur - aprev;
  mult = angle / cur;
  angle = mult * now + aprev;

  while(angle > 180.0f)
    angle -= 360.0f;

  if((angle - angle_prev < 0.0f && angle - angle_prev > -90.0f) || angle - angle_prev > 90.0f)
  {
    angle = angle_prev;
  }
  angle_prev = angle;

  return angle;
}

inline float csps_getangle23from14(float angle)
{
  if(angle > 0.0f) angle -= 180.0f;
  else angle += 180.0f;
  return angle;
}

inline float csps_getrpm(void)
{
  return csps_rpm;
}

inline float csps_getuspa(void)
{
  return csps_uspa;
}

inline float csps_getperiod(void)
{
  return csps_period;
}

inline uint8_t csps_isrotates(void)
{
  return csps_rotates;
}

inline uint8_t csps_isfound(void)
{
  return csps_found;
}

inline uint8_t csps_iserror(void)
{
  return csps_errors > 3.0f;
}

inline void csps_loop(void)
{
  static uint32_t last_error_null = 0;

  uint32_t pulse_last = csps_pulse_last;
  uint32_t last = csps_last;
  uint32_t now = Delay_Tick;

  if(DelayDiff(now, pulse_last) > 50000)
  {
    pulse_last = now;
    for(int i = 0; i < IRQ_SIZE; i++)
      cspc_irq_data[i] = 0;
    csps_found = 0;
    csps_rpm = 0;
    csps_period = 1.0f / csps_rpm;
  }
  else if(DelayDiff(now, last) > 3000000)
  {
    last = now;
    csps_found = 0;
    csps_rpm = 0;
    csps_rotates = 0;
    csps_period = 1.0f / csps_rpm;
  }

  if(DelayDiff(now, last_error_null) > 50000)
  {
    csps_errors *= 0.95f;
    last_error_null = now;
  }

}

