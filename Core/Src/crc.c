#include "crc.h"

#ifdef CRC_SW

#define POLY 0x8005 // CRC-16-MAXIM (IBM) (or 0xA001)

uint16_t CRC16_Generate(uint8_t * input, uint32_t size)
{
    uint8_t i,j;
    uint16_t data;
    uint16_t crc = 0x0000;//0xFFFF;

    for (j = 0; j < size; j++)
    {
        for (i=0, data=(uint8_t)0xff & input[j];
                i < 8;
                i++, data >>= 1)
        {
            if ((crc & 0x0001) ^ (data & 0x0001))
            {
                crc = (crc >> 1) ^ POLY;
            }
            else  crc >>= 1;
        }
    }

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);

    return (crc);
}

#elif defined(CRC_HW)

static CRC_HandleTypeDef * handle_crc;

void CRC16_RegisterHardware(CRC_HandleTypeDef * hcrc)
{
  handle_crc = hcrc;
}

inline uint16_t CRC16_Generate(uint8_t * input, uint32_t size)
{
  uint16_t result = 0;
  if(handle_crc != NULL)
  {
      result = HAL_CRC_Calculate(handle_crc, (uint32_t*)input, size);
  }
  return result;
}

inline uint8_t CRC8_Generate(uint8_t * input, uint32_t size)
{
  uint16_t result = 0;
  if(handle_crc != NULL)
  {
    result = HAL_CRC_Calculate(handle_crc, (uint32_t*)input, size);

  }
  return (result & 0xFF) ^ (result >> 8);
}

#endif
