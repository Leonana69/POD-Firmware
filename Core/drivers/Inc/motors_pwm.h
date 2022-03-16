#ifndef __MOTORS_PWM_H__
#define __MOTORS_PWM_H__

#include <stdbool.h>
#include <stdint.h>

void motorsPwmInit();
bool motorsPwmTest();
void motorsPwmSetRatio(uint8_t id, uint16_t thrust);
uint16_t motorsPwmGetValue(uint8_t id);

#endif //__MOTORS_PWM_H__
