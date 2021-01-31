/*
 * packets.h
 *
 *  Created on: 31 янв. 2021 г.
 *      Author: VHEMaster
 */

#ifndef PACKETS_H_
#define PACKETS_H_

#include "main.h"

#define PK_GeneralStatusRequestID 1
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;

}PK_GeneralStatusRequest_t;

#define PK_GeneralStatusResponseID 2
typedef struct
{
  uint8_t PacketID;
  uint8_t PacketLength;
  uint8_t Destination;
  uint8_t Dummy;
  float RPM;
  float Pressure;
  float Load;
  float IgnitionAngle;
}PK_GeneralStatusResponse_t;

extern int8_t PK_Copy(void * dest, void * source);

#define PACKET_C(x) extern x##_t x;

PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);


#endif /* PACKETS_H_ */
