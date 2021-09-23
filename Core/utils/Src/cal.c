#include "cal.h"

uint8_t calculate_cksum(void* data, size_t len) {
  unsigned char* c = data;
  unsigned char cksum = 0;
  
  for (int i = 0; i < len; i++)
    cksum += *(c++);

  return cksum;
}