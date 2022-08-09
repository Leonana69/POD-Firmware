#define DEBUG_MODULE "FDIS"

#include "front_dis.h"
#include "opt3101.h"
#include "_i2c.h"
#include "debug.h"
#include "config.h"
#include "static_mem.h"
#include "system.h"

static bool isInit = false;
static bool opt3101Present = false;

static void frontDisTask();
STATIC_MEM_TASK_ALLOC(frontDisTask, FRONT_DIS_TASK_STACKSIZE);

static I2CDrv *I2Cx;
static opt3101_dev opt3101;
static uint8_t opt3101_addr = OPT3101_CHIP_ADDR;

static int16_t dis3ch[3];

static void sensorsMsDelay(uint32_t period) {
    osDelay(period);
}

static uint16_t sensorsGetMilli() {
    return (uint16_t)osKernelGetTickCount();
}

void frontDisInit() {
    if (isInit)
        return;

    I2Cx = &sensorI2C;

    if (HAL_I2C_IsDeviceReady(I2Cx->hi2c, opt3101_addr << 1, 3, 100) != HAL_OK) {
		DEBUG_PRINT("OPT3101 not found.\n");
		return;
	}

    opt3101Present = true;

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
        STATIC_MEM_TASK_CREATE(frontDisTask, frontDisTask, FRONT_DIS_TASK_NAME, NULL, FRONT_DIS_TASK_PRI);
    } else {
        DEBUG_PRINT("Opt3101 config [FAILED]\n");
        return;
    }
    isInit = true;
}

bool frontDisTest() {
    return (!opt3101Present) || isInit;
}

void frontDisTask() {
    systemWaitStart();
    int cnt = 0;
    while (1) {
        opt3101Sample(&opt3101);
        dis3ch[opt3101.channelUsed] = opt3101.distanceMillimeters;
        if (opt3101.channelUsed == 2 && cnt++ == 20) {
            cnt = 0;
            DEBUG_PRINT("%d %d %d\n", dis3ch[0], dis3ch[1], dis3ch[2]);
        }
    }
}