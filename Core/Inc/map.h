/*
 * map.h
 *
 *  Created on: 10 янв. 2021 г.
 *      Author: VHEMaster
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

#include "main.h"

extern void map_init(void);
extern void map_adc_read(void);
extern uint8_t map_getraw(void);
extern float map_getpressure(void);
extern uint8_t map_iserror(void);


#endif /* INC_MAP_H_ */
