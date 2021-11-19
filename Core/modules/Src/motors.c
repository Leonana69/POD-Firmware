#include "motors.h"
#include "config.h"
#include "motors_pwm.h"
#include "motors_dshot.h"
#include "debug.h"

static MotorsType currentMotors = MOTORS_TYPE;

typedef struct {
	void (*init)();
	bool (*test)();
	void (*setRatio)(uint8_t id, uint16_t thrust);
	const char* name;
} Motors;

static Motors motorsFunctions[] = {
  { .init = motorsPwmInit, .test = motorsPwmTest, .setRatio = motorsPwmSetRatio, .name = "PWM" },
  { .init = motorsDshotInit, .test = motorsDshotTest, .setRatio = motorsDshotSetRatio, .name = "DSHOT" },
};

void motorsInit() {
	motorsFunctions[currentMotors].init();
	DEBUG_PRINT("Using %s (%d) motors.\n", motorsGetName(), currentMotors);
}

bool motorsTest() {
	return motorsFunctions[currentMotors].test();
}

void motorsSetRatio(uint8_t id, uint16_t thrust) {
	motorsFunctions[currentMotors].setRatio(id, thrust);
}

const char* motorsGetName() {
	return motorsFunctions[currentMotors].name;
}
