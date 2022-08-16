#ifndef __VL53L1_H__
#define __VL53L1_H__

#include <stdbool.h>
#include <stdint.h>
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "vl53l1_register_settings.h"

#define expCoeff 2.92135f
#define expPointA 2.5f
#define expStdA 0.0025f
#define RANGE_OUTLIER_LIMIT 5000
#define VL53L1_DEV_ADDR_DEFAULT 0x29
int8_t vl53l1Init(VL53L1_Dev_t *dev);
bool vl53l1Test(VL53L1_Dev_t *dev);

#endif