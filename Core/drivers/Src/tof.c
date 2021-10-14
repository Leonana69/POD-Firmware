#define DEBUG_MODULE "TOF"

#include "tof.h"

#include "debug.h"
#include "config.h"
#include "vl53l1_platform.h"

static bool isInit = false;

void tofInit() {
	if (isInit)
		return;

	isInit = true;
}

bool tofTest() {
	return isInit;
}

/*! vl53l1 platform functions */

/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data) {
	
}