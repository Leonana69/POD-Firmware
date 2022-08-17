#define DEBUG_MODULE "FDIS"

#include "front_dis.h"
#include "opt3101.h"
#include "_i2c.h"
#include "debug.h"
#include "config.h"
#include "static_mem.h"
#include "system.h"
#include "tof.h"
#include "stabilizer_types.h"
#include "config.h"
#include "vl53l1.h"

static bool isInit = false;
static bool frontDisPresent = false;

static void frontDisTask();
STATIC_MEM_TASK_ALLOC(frontDisTask, FRONT_DIS_TASK_STACKSIZE);

static I2CDrv *I2Cx;
#if (FRONT_DIS_TYPE == FRONT_DIS_OPT3101)
static opt3101_dev opt3101;
static uint8_t opt3101_addr = OPT3101_CHIP_ADDR;
int16_t dis3ch[3];
#elif (FRONT_DIS_TYPE == FRONT_DIS_VL53L1X)
static VL53L1_Dev_t vl53l1Dev;
uint16_t frontDis = 0;
#define FRONT_DIS_RATE RATE_25_HZ
#endif

static void sensorsMsDelay(uint32_t period) {
    osDelay(period);
}

static uint16_t sensorsGetMilli() {
    return (uint16_t)osKernelGetTickCount();
}

void frontDisInit() {
    if (isInit || FRONT_DIS_TYPE == FRONT_DIS_NONE)
        return;

    I2Cx = &sensorI2C;
    
    #if (FRONT_DIS_TYPE == FRONT_DIS_VL53L1X)
        if (HAL_I2C_IsDeviceReady(I2Cx->hi2c, VL53L1_DEV_ADDR_DEFAULT << 1, 3, 100) != HAL_OK) {
            DEBUG_PRINT("VL53L1X not found.\n");
            return;
        }

        vl53l1Dev.read = i2cFrontDisRead;
        vl53l1Dev.write = i2cFrontDisWrite;
        vl53l1Dev.delay = sensorsMsDelay;
        vl53l1Dev.millis = sensorsGetMilli;
        vl53l1Dev.i2c_slave_address = VL53L1_DEV_ADDR_DEFAULT;
        if (vl53l1Init(&vl53l1Dev) == 0) {
            DEBUG_PRINT("VL53L1 init [OK]\n");
        } else {
            DEBUG_PRINT("VL53L1 init [FAILED]\n");
            return;
        }
    #elif (FRONT_DIS_TYPE == FRONT_DIS_OPT3101)
        if (HAL_I2C_IsDeviceReady(I2Cx->hi2c, opt3101_addr << 1, 3, 100) != HAL_OK) {
            DEBUG_PRINT("OPT3101 not found.\n");
            return;
        }
        opt3101.intf_ptr = &opt3101_addr;
        opt3101.read = i2cSensorsRead;
        opt3101.write = i2cSensorsWrite;
        opt3101.millis = sensorsGetMilli;
        opt3101.delay = sensorsMsDelay;

        if (opt3101Init(&opt3101) != 0) {
            DEBUG_PRINT("OPT3101 init [FAILED]\n");
            return;
        }
        int8_t rslt = 0;
        rslt |= opt3101SetFrameTiming(&opt3101, 64);
        rslt |= opt3101SetChannel(&opt3101, OPT3101_CHANNEL_AUTO);
        rslt |= opt3101SetBrightness(&opt3101, BR_HIGH);

        if (rslt == 0) {
            DEBUG_PRINT("OPT3101 init [OK]\n");
        } else {
            DEBUG_PRINT("Opt3101 config [FAILED]\n");
            return;
        }
    #endif 
    frontDisPresent = true;

    STATIC_MEM_TASK_CREATE(frontDisTask, frontDisTask, FRONT_DIS_TASK_NAME, NULL, FRONT_DIS_TASK_PRI);
    isInit = true;
}

bool frontDisTest() {
    return (!frontDisPresent) || isInit;
}

void frontDisTask() {
    systemWaitStart();
#if (FRONT_DIS_TYPE == FRONT_DIS_VL53L1X)
    uint32_t lastWakeTime = osKernelGetTickCount();
	uint32_t wakeDelay = osKernelGetTickFreq() / FRONT_DIS_RATE;
	VL53L1_RangingMeasurementData_t vl53l1RangingData;
#endif

    while (1) {
    #if (FRONT_DIS_TYPE == FRONT_DIS_OPT3101)
        opt3101Sample(&opt3101);
        dis3ch[opt3101.channelUsed] = opt3101.distanceMillimeters;
        if (opt3101.channelUsed == 2 && cnt++ == 20) {
            cnt = 0;
            DEBUG_PRINT("%d %d %d\n", dis3ch[0], dis3ch[1], dis3ch[2]);
        }
    #elif (FRONT_DIS_TYPE == FRONT_DIS_VL53L1X)
        lastWakeTime += wakeDelay;
		osDelayUntil(lastWakeTime);
		VL53L1_WaitMeasurementDataReady(&vl53l1Dev);
		VL53L1_GetRangingMeasurementData(&vl53l1Dev, &vl53l1RangingData);
		frontDis = vl53l1RangingData.RangeMilliMeter;
		VL53L1_ClearInterruptAndStartMeasurement(&vl53l1Dev);
    #endif
    }
}