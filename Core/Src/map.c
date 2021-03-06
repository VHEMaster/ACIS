/*
 * map.c
 *
 *  Created on: 10 янв. 2021 г.
 *      Author: VHEMaster
 */

#include "map.h"

#define multiplier 1.0f
#define lowpass_koff 0.02f
static volatile float map_data = 101325.0f;
static volatile uint8_t map_error = 0;
static volatile uint8_t map_raw = 0;
extern TIM_HandleTypeDef htim8;

#define REF_VOLTAGE 3.32f
#define RES_DIVIDER 1.467f


void map_init(void)
{
  HAL_GPIO_WritePin(ADC_FORMAT_GPIO_Port, ADC_FORMAT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_BPO_GPIO_Port, ADC_BPO_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_nCE_GPIO_Port, ADC_nCE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_nCS_GPIO_Port, ADC_nCS_Pin, GPIO_PIN_RESET);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

inline void map_adc_read(void)
{
  uint8_t portdata = GPIOA->IDR & 0xFF;
  map_raw = portdata;
  float voltage = portdata / 50.0f;
  if(voltage < 0.1f) map_error = 1;
  else map_error = 0;
  float pressure = (voltage - 0.5f) * 25517.0f;
  if(pressure < 0.0f)
    pressure = 0.0f;
  map_data = map_data * (1.0f - lowpass_koff) + pressure * lowpass_koff;
}

inline float map_getpressure(void)
{
  return map_data;
}

inline uint8_t map_getraw(void)
{
  return map_raw;
}

inline uint8_t map_iserror(void)
{
  return map_error;
}
