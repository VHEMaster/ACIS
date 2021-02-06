/*
 * config.h
 *
 *  Created on: 2 февр. 2021 г.
 *      Author: VHEMaster
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include "packets.h"

extern HAL_StatusTypeDef config_load(sAcisConfig * config);
extern HAL_StatusTypeDef config_default(sAcisConfig * config);
extern HAL_StatusTypeDef config_save(sAcisConfig * config);

#endif /* INC_CONFIG_H_ */
