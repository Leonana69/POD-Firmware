#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdbool.h>
#include <stdint.h>

/*! do not use enum here */
#define MOTORS_PWM    0
#define MOTORS_DSHOT  1

#define NBR_OF_MOTORS 4
/*! Motors IDs define */
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

void motorsInit();
bool motorsTest();
void motorsSetRatio(uint8_t id, uint16_t thrust);

#endif //__MOTORS_H__
