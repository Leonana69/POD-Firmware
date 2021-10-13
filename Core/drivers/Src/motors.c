#include "motors.h"
#include "tim.h"
#include "config.h"
#include "cfassert.h"
#include "debug.h"

static bool isInit = false;

struct {
	TIM_HandleTypeDef* tim;
	uint32_t channel;
} MotorTim[4];

static uint16_t motorsThrustToPulse(uint16_t thrust) {
  return ((thrust) >> (16 - MOTORS_TIM_BITS) & ((1 << MOTORS_TIM_BITS) - 1));
}

void motorsInit() {
	if (isInit)
		return;
	MotorTim[0].tim = &MOTOR1TIM;
	MotorTim[0].channel = MOTOR1CHANNEL;
	MotorTim[1].tim = &MOTOR2TIM;
	MotorTim[1].channel = MOTOR2CHANNEL;
	MotorTim[2].tim = &MOTOR3TIM;
	MotorTim[2].channel = MOTOR3CHANNEL;
	MotorTim[3].tim = &MOTOR4TIM;
	MotorTim[3].channel = MOTOR4CHANNEL;
	for (int i = 0; i < NBR_OF_MOTORS; i++)
		HAL_TIM_PWM_Start(MotorTim[i].tim, MotorTim[i].channel);
	isInit = true;
}
bool motorsTest() {
	return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ithrust) {
	if (isInit) {
		ASSERT(id < NBR_OF_MOTORS);
		uint16_t ratio = ithrust;
		__HAL_TIM_SET_COMPARE(MotorTim[id].tim, MotorTim[id].channel, motorsThrustToPulse(ratio));
	}
}