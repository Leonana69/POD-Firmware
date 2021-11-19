#include "motors_pwm.h"
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

static void motorsBeep(uint8_t id, uint16_t cycle, uint16_t ratio, bool enable);

void motorsPwmInit() {
	if (isInit)
		return;
	MotorTim[0].tim = &MOTOR1_TIM;
	MotorTim[0].channel = MOTOR1_CHANNEL;
	MotorTim[1].tim = &MOTOR2_TIM;
	MotorTim[1].channel = MOTOR2_CHANNEL;
	MotorTim[2].tim = &MOTOR3_TIM;
	MotorTim[2].channel = MOTOR3_CHANNEL;
	MotorTim[3].tim = &MOTOR4_TIM;
	MotorTim[3].channel = MOTOR4_CHANNEL;
	for (int i = 0; i < NBR_OF_MOTORS; i++)
		HAL_TIM_PWM_Start(MotorTim[i].tim, MotorTim[i].channel);
	isInit = true;
}

bool motorsPwmTest() {
	/*! A4, A5, F5, D5 */
	int testTone[4] = { 440, 880, 698, 587 };
	int motorsTimBeepClkFreq = 84000000L / 5;
	for (uint8_t i = 0; i < NBR_OF_MOTORS; i++) {
		motorsBeep(i, (uint16_t)(motorsTimBeepClkFreq / testTone[i]), (uint16_t)(motorsTimBeepClkFreq / 440)/ 20, true);
		osDelay(50);
		motorsBeep(i, 0, 0, false);
		osDelay(150);
	}
	return isInit;
}

void motorsBeep(uint8_t id, uint16_t cycle, uint16_t ratio, bool enable) {
	ASSERT(id < NBR_OF_MOTORS);
	HAL_TIM_PWM_Stop(MotorTim[id].tim, MotorTim[id].channel);
	MotorTim[id].tim->Init.Prescaler = 0;
	MotorTim[id].tim->Init.Period = 255;
	if (enable) {
		MotorTim[id].tim->Init.Prescaler = (5 - 1);
		MotorTim[id].tim->Init.Period = cycle;
	}

	HAL_TIM_PWM_Init(MotorTim[id].tim);
	HAL_TIM_PWM_Start(MotorTim[id].tim, MotorTim[id].channel);
	__HAL_TIM_SET_COMPARE(MotorTim[id].tim, MotorTim[id].channel, ratio);
}

static uint16_t motorsThrustToPulse(uint16_t thrust) {
  return ((thrust) >> (16 - MOTORS_TIM_BITS) & ((1 << MOTORS_TIM_BITS) - 1));
}

void motorsPwmSetRatio(uint8_t id, uint16_t ithrust) {
	if (isInit) {
		ASSERT(id < NBR_OF_MOTORS);
		uint16_t ratio = ithrust;
		__HAL_TIM_SET_COMPARE(MotorTim[id].tim, MotorTim[id].channel, motorsThrustToPulse(ratio));
	}
}