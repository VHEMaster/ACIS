#include "main.h"

#ifndef INC_CRC_H_
#define INC_CRC_H_


#define CRC_HW
//#define CRC_SW


extern uint8_t CRC8_Generate(uint8_t * input, uint32_t size);
extern uint16_t CRC16_Generate(uint8_t * input, uint32_t size);

#ifdef CRC_HW
extern void CRC16_RegisterHardware(CRC_HandleTypeDef * hcrc);
#elif !defined(CRC_SW)
#error Must be defined CRC_HW or CRC_SW
#endif

#endif /* INC_CRC_H_ */
