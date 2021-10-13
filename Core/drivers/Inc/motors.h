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

/*! Motors test define */
#define MOTORS_TEST_RATIO (uint16_t)(0.2 * (1 << 16))
#define MOTORS_TEST_ON_TIME_MS 50
#define MOTORS_TEST_DELAY_TIME_MS 150

void motorsInit();
bool motorsTest();
void motorsSetRatio(uint32_t id, uint16_t ithrust);

#endif //__MOTORS_H__
