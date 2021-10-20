#ifndef __FLOW_H__
#define __FLOW_H__

#include <stdint.h>

typedef struct motionBurst_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0    : 1;
      uint8_t runMode       : 2;
      uint8_t reserved1     : 1;
      uint8_t rawFrom0      : 1;
      uint8_t reserved2     : 2;
      uint8_t motionOccured : 1;
    };
  };

  int16_t deltaX;
  int16_t deltaY;
  uint8_t squal;
  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;
  uint16_t shutter;

} __attribute__((packed)) motionBurst_t;

void flowInit();
bool flowTest();

#endif //__FLOW_H__
