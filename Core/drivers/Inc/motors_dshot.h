#ifndef __MOTORS_DSHOT_H__
#define __MOTORS_DSHOT_H__

#include <stdbool.h>
#include <stdint.h>

void motorsDshotInit();
bool motorsDshotTest();
void motorsDshotSetRatio(uint8_t id, uint16_t thrust);

#endif //__MOTORS_DSHOT_H__
