#include "motors.h"
#include "config.h"
#include "motors_pwm.h"
#include "motors_dshot.h"
#include "debug.h"

typedef struct {
	void (*init)();
	bool (*test)();
	void (*setRatio)(uint8_t id, uint16_t thrust);
	const char* name;
} Motors;

static Motors motorsFunctions = {
#if (MOTORS_TYPE == MOTORS_PWM)
  .init = motorsPwmInit, .test = motorsPwmTest, .setRatio = motorsPwmSetRatio, .name = "PWM",
#elif (MOTORS_TYPE == MOTORS_DSHOT)
  .init = motorsDshotInit, .test = motorsDshotTest, .setRatio = motorsDshotSetRatio, .name = "DSHOT",
#endif
};

void motorsInit() {
	motorsFunctions.init();
	DEBUG_PRINT("Using %s (%d) motors.\n", motorsFunctions.name, MOTORS_TYPE);
}

bool motorsTest() {
	return motorsFunctions.test();
}

void motorsSetRatio(uint8_t id, uint16_t thrust) {
	motorsFunctions.setRatio(id, thrust);
}
