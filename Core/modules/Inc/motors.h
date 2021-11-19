#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdbool.h>
#include <stdint.h>

#define NBR_OF_MOTORS 4
/*! Motors IDs define */
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

typedef enum {
  MOTORS_PWM,
  MOTORS_DSHOT,
  MOTORS_COUNT,
} MotorsType;

void motorsInit();
bool motorsTest();
void motorsSetRatio(uint8_t id, uint16_t thrust);
const char* motorsGetName();

#endif //__MOTORS_H__
