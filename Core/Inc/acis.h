/*
 * acis.h
 *
 *  Created on: 10 янв. 2021 г.
 *      Author: VHEMaster
 */

#ifndef INC_ACIS_H_
#define INC_ACIS_H_

#include "main.h"
#include "xCommand.h"

extern void acis_loop(void);
extern void acis_loop_irq(void);
extern void acis_hall_exti(void);
extern void acis_init(void);
extern void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length);

#endif /* INC_ACIS_H_ */
