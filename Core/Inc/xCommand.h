/*
 * xCommand.h
 *
 *  Created on: Dec 6, 2020
 *      Author: denys.prokhorov
 */


#ifndef XCOMMAND_H_
#define XCOMMAND_H_

#include "main.h"

#define HEADER_ACK_BIT (1<<7)
#define HEADER_ALARM_BITS ((1<<7)|(1<<6))
#define TASK_SLEEP  { osDelay(1); } // Task must give it's time to another process or just skip some time
#define MAX_PACK_LEN (384)

typedef enum {
    etrNone,
    etrPC,
    etrACIS,
    etrCTRL,
    etrCount
} eTransChannels;

extern void xFifosInit(void);
extern void xGetterTask(void * arg);
extern void xGetterInit(void);
extern void xGetterLoop(void);
extern void xDmaTxIrqHandler(UART_HandleTypeDef *huart);
extern void xDmaErIrqHandler(UART_HandleTypeDef *huart);
extern int8_t xSender(eTransChannels xChaDest, uint8_t* xMsgPtr, uint32_t xMsgLen);

#endif /* XCOMMAND_H_ */
