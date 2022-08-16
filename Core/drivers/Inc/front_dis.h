#ifndef __FRONT_DIS_H__
#define __FRONT_DIS_H__

#include <stdbool.h>

#define FRONT_DIS_NONE 0
#define FRONT_DIS_OPT3101 1
#define FRONT_DIS_VL53L1X 2

void frontDisInit();
bool frontDisTest();

#endif //__FRONT_DIS_H__
