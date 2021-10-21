#define DEBUG_MODULE "SELFT"

#include "config.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "sensors.h"
#include "pm.h"
#include "static_mem.h"
#include "cal.h"

#define PROP_TEST_DIM 100
/*! PROP_TEST define */
#define PROP_TEST_RATIO 0xFFFF
#define PROP_TEST_ON_TIME_MS 20
#define PROP_TEST_DELAY_TIME_MS 250

NO_DMA_CCM_SAFE_ZERO_INIT static float accelX[PROP_TEST_DIM];
NO_DMA_CCM_SAFE_ZERO_INIT static float accelY[PROP_TEST_DIM];
NO_DMA_CCM_SAFE_ZERO_INIT static float accelZ[PROP_TEST_DIM];

static float accelVarBaseX;
static float accelVarBaseY;
static float accelVarBaseZ;

static float accelVarX[NBR_OF_MOTORS];
static float accelVarY[NBR_OF_MOTORS];
static float accelVarZ[NBR_OF_MOTORS];

typedef enum {
	TEST_ACCEL_CONFIG,
	TEST_BASE_NOISE_MES,
	TEST_PROP_MES,
	TEST_BATTERY,
	TEST_DONE,
} SelfTestState;

static int8_t currentState = TEST_ACCEL_CONFIG;

bool selfTestPassed() {
	return currentState == TEST_DONE;
}

void selfTestRun(sensorData_t *sensors) {
	static int loopCount = 0;
	static int curMotor = 0;
	switch (currentState) {
		case TEST_ACCEL_CONFIG:
			// sensorsSetAccelMode(ACCEL_MODE_PROPTEST);
			currentState = TEST_BASE_NOISE_MES;
			break;
		case TEST_BASE_NOISE_MES:
			accelX[loopCount] = sensors->accel.x;
			accelY[loopCount] = sensors->accel.y;
			accelZ[loopCount] = sensors->accel.z;
			if (++loopCount >= PROP_TEST_DIM) {
				loopCount = 0;
				accelVarBaseX = fVariance(accelX, PROP_TEST_DIM);
				accelVarBaseY = fVariance(accelY, PROP_TEST_DIM);
				accelVarBaseZ = fVariance(accelZ, PROP_TEST_DIM);
				DEBUG_PRINT("Accel base noise: X+Y=%.6f, Z=%.6f\n", accelVarBaseX + accelVarBaseY, accelVarBaseZ);
				currentState = TEST_PROP_MES;
			}
			break;
		case TEST_PROP_MES:
			if (loopCount < PROP_TEST_DIM) {
				accelX[loopCount] = sensors->accel.x;
				accelY[loopCount] = sensors->accel.y;
				accelZ[loopCount] = sensors->accel.z;
			} else if (loopCount == PROP_TEST_DIM) {
				accelVarX[curMotor] = fVariance(accelX, PROP_TEST_DIM);
				accelVarY[curMotor] = fVariance(accelY, PROP_TEST_DIM);
				accelVarZ[curMotor] = fVariance(accelZ, PROP_TEST_DIM);
				DEBUG_PRINT("Accel of motor %d: X+Y=%.6f, Z=%.6f\n", curMotor + 1, accelVarX[curMotor] + accelVarY[curMotor], accelVarZ[curMotor]);
			}
			
			loopCount++;

			if (loopCount == 1) {
				motorsSetRatio(curMotor, PROP_TEST_RATIO);
			} else if (loopCount == PROP_TEST_ON_TIME_MS) {
				motorsSetRatio(curMotor, 0);
			} else if (loopCount >= PROP_TEST_DELAY_TIME_MS) {
				curMotor++;
				loopCount = 0;
			}

			if (curMotor == NBR_OF_MOTORS) {
				loopCount = 0;
				DEBUG_PRINT("Self test done.\n");
				currentState = TEST_DONE;
				// sensorsSetAccelMode(ACCEL_MODE_FLIGHT);
			}

			break;
		case TEST_BATTERY:
		case TEST_DONE:
		default:
		break;
	}
}
