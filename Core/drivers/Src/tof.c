#define DEBUG_MODULE "TOF"

#include <math.h>

#include "tof.h"
#include "_i2c.h"
#include "debug.h"
#include "config.h"
#include "static_mem.h"
#include "system.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "log.h"
#include "vl53l1.h"

static bool isInit = false;
static bool vl53l1Present = false;
static I2CDrv *I2Cx;
#define TOF_RATE RATE_25_HZ
static void tofTask();
STATIC_MEM_TASK_ALLOC(tofTask, TOF_TASK_STACKSIZE);

static VL53L1_Dev_t vl53l1Dev;
static uint16_t rangeLast;

static void sensorsMsDelay(uint32_t period) {
    osDelay(period);
}

static uint16_t sensorsGetMilli() {
    return (uint16_t)osKernelGetTickCount();
}

void tofInit() {
	if (isInit)
		return;
	I2Cx = &tofI2C;

	if (HAL_I2C_IsDeviceReady(I2Cx->hi2c, VL53L1_DEV_ADDR_DEFAULT << 1, 3, 100) != HAL_OK) {
		DEBUG_PRINT("VL53L1X not found.\n");
		return;
	}
	vl53l1Present = true;

	vl53l1Dev.read = i2cTofRead;
	vl53l1Dev.write = i2cTofWrite;
	vl53l1Dev.delay = sensorsMsDelay;
	vl53l1Dev.millis = sensorsGetMilli;
	vl53l1Dev.i2c_slave_address = VL53L1_DEV_ADDR_DEFAULT;

	if (vl53l1Init(&vl53l1Dev) == 0) {
		DEBUG_PRINT("VL53L1 init [OK]\n");
		STATIC_MEM_TASK_CREATE(tofTask, tofTask, TOF_TASK_NAME, NULL, TOF_TASK_PRI);
		isInit = true;
	} else {
		DEBUG_PRINT("VL53L1 init [FAILED]\n");
        return;
	}
}

bool tofTest() {
	return (!vl53l1Present) || (isInit & vl53l1Test(&vl53l1Dev));
}

void tofTask() {
	systemWaitStart();
	uint32_t lastWakeTime = osKernelGetTickCount();
	uint32_t wakeDelay = osKernelGetTickFreq() / TOF_RATE;
	tofMeasurement_t tofData;
	VL53L1_RangingMeasurementData_t vl53l1RangingData;
	
	while (1) {
		lastWakeTime += wakeDelay;
		osDelayUntil(lastWakeTime);
		VL53L1_WaitMeasurementDataReady(&vl53l1Dev);
		VL53L1_GetRangingMeasurementData(&vl53l1Dev, &vl53l1RangingData);
		tofData.distance = vl53l1RangingData.RangeMilliMeter;
		rangeLast = vl53l1RangingData.RangeMilliMeter;
		VL53L1_ClearInterruptAndStartMeasurement(&vl53l1Dev);

		if (tofData.distance < RANGE_OUTLIER_LIMIT) {
			tofData.timestamp = osKernelGetTickCount();
			tofData.distance = tofData.distance * 0.001f;
			tofData.stdDev = expStdA * (1.0f  + expf(expCoeff * (tofData.distance - expPointA)));
			
			estimatorEnqueueTOF(&tofData);
		}
	}
}

LOG_GROUP_START(tof)
/**
 * @brief True if motion occured since the last measurement
 */
LOG_ADD(LOG_UINT16, distance, &rangeLast)
LOG_GROUP_STOP(tof)
