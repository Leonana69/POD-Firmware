#ifndef __MOTORS_DSHOT_H__
#define __MOTORS_DSHOT_H__

#include <stdbool.h>
#include <stdint.h>

void motorsDshotInit();
bool motorsDshotTest();
void motorsDshotSetRatio(uint8_t id, uint16_t thrust);
uint16_t motorsDshotGetValue(uint8_t id);

#endif //__MOTORS_DSHOT_H__
