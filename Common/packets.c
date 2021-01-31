#include "packets.h"


#define PACKET_C(x) x##_t x = {x##ID,sizeof(x##_t),0,0}

PACKET_C(PK_GeneralStatusRequest);
PACKET_C(PK_GeneralStatusResponse);


int8_t PK_Copy(void * dest, void * source)
{
  if(dest == 0 || source == 0) return -1;
  uint8_t * dest_data = (uint8_t*)dest;
  uint8_t * source_data = (uint8_t*)source;
  int8_t length = dest_data[1];
  if(dest_data[1] != source_data[1] || dest_data[0] != source_data[0] || length == 0 || length > 384) return -2;
  for(uint8_t i=2;i<length+2;i++)
    *dest_data++ = *source_data++;
  return length;
}
