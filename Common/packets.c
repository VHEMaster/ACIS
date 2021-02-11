#include "packets.h"


#define PACKET_C(x) x##_t x __attribute__((aligned(32))) = {x##ID,sizeof(x##_t),0,0}

PACKET_C(PK_Ping);
PACKET_C(PK_Pong);
PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);
PACKET_C(PK_TableMemoryRequest);
PACKET_C(PK_TableMemoryData);
PACKET_C(PK_TableMemoryAcknowledge);
PACKET_C(PK_ConfigMemoryRequest);
PACKET_C(PK_ConfigMemoryData);
PACKET_C(PK_ConfigMemoryAcknowledge);
PACKET_C(PK_SaveConfig);
PACKET_C(PK_RestoreConfig);
PACKET_C(PK_SaveConfigAcknowledge);
PACKET_C(PK_RestoreConfigAcknowledge);
PACKET_C(PK_DragStart);
PACKET_C(PK_DragUpdateRequest);
PACKET_C(PK_DragUpdateResponse);
PACKET_C(PK_DragStop);
PACKET_C(PK_DragPointRequest);
PACKET_C(PK_DragPointResponse);
PACKET_C(PK_DragStartAcknowledge);
PACKET_C(PK_DragStopAcknowledge);
PACKET_C(PK_PcConnected);
PACKET_C(PK_FuelSwitch);


int16_t PK_Copy(void * dest, void * source)
{
  if(dest == 0 || source == 0) return -1;
  uint8_t * dest_data = (uint8_t*)dest;
  uint8_t * source_data = (uint8_t*)source;
  int16_t length = dest_data[1];
  if(dest_data[1] != source_data[1] || dest_data[0] != source_data[0] || length == 0 || length > 384) return -2;
  for(uint8_t i=2;i<length+2;i++)
    *dest_data++ = *source_data++;
  return length;
}
