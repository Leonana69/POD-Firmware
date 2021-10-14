#include "motors.h"
#include "tim.h"
#include "config.h"
#include "cfassert.h"
#include "debug.h"
#include "cmsis_os2.h"
#include "tim.h"

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

void motorsSetRatio(uint32_t id, uint16_t ithrust) {
	if (isInit) {
		ASSERT(id < NBR_OF_MOTORS);
		uint16_t ratio = ithrust;
		__HAL_TIM_SET_COMPARE(MotorTim[id].tim, MotorTim[id].channel, motorsThrustToPulse(ratio));
	}
}

#define MOTORS_TIM_BEEP_CLK_FREQ  (84000000L / 5)
#define A4    440
#define A5    880
#define F5    698
#define D5    587

int testTone[4] = { A4, A5, F5, D5 };
bool motorsTest() {
	for (int i = 0; i < NBR_OF_MOTORS; i++) {
		motorsBeep(i, testTone[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20, true);
		osDelay(50);
		motorsBeep(i, 0, 0, false);
		osDelay(150);
	}
	return isInit;
}

void motorsBeep(uint32_t id, uint16_t frequency, uint16_t ratio, bool enable) {
	ASSERT(id < NBR_OF_MOTORS);
	HAL_TIM_PWM_Stop(MotorTim[id].tim, MotorTim[id].channel);
  MotorTim[id].tim->Init.Prescaler = 0;
  MotorTim[id].tim->Init.Period = 255;
  if (enable) {
    MotorTim[id].tim->Init.Prescaler = (5 - 1);
    MotorTim[id].tim->Init.Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
  }

	HAL_TIM_PWM_Init(MotorTim[id].tim);
	HAL_TIM_PWM_Start(MotorTim[id].tim, MotorTim[id].channel);
  __HAL_TIM_SET_COMPARE(MotorTim[id].tim, MotorTim[id].channel, ratio);
}
