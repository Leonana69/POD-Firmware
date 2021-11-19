#include "motors_dshot.h"
#include "tim.h"
#include "config.h"
#include "cmsis_os2.h"
#include "cfassert.h"
#include "debug.h"
#include "motors.h"

struct {
	TIM_HandleTypeDef* tim;
	uint32_t channel;
} static MotorTim[4];

static bool isInit = false;

void motorsDshotInit() {
	if (isInit)
		return;
	
	isInit = true;
}

bool motorsDshotTest() {
	return isInit;
}

void motorsDshotSetRatio(uint8_t id, uint16_t ithrust) {
	if (isInit) {
		
	}
}