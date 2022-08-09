#include "pid.h"
#include "stabilizer_types.h"
#include "cal.h"
#include "controller_pid_attitude.h"
#include "debug.h"

static CascadePidObject roll;
static CascadePidObject pitch;
static CascadePidObject yaw;
/** smaller pid value for high quality motors */
static CascadePidParam paramRoll = {
	.val = {
		.kp = 5.0,
		.ki = 2.0,
		.kd = 0.0,
		.rate = ATTITUDE_RATE,
		.iLimit = 20.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 15.0,
	},
	.rate = {
		.kp = 150.0,
		.ki = 200.0,
		.kd = 1.0,
		.rate = ATTITUDE_RATE,
		.iLimit = 33.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 30.0,
	}
};

static CascadePidParam paramPitch = {
	.val = {
		.kp = 5.0,
		.ki = 2.0,
		.kd = 0.0,
		.rate = ATTITUDE_RATE,
		.iLimit = 20.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 15.0,
	},
	.rate = {
		.kp = 150.0,
		.ki = 200.0,
		.kd = 1.0,
		.rate = ATTITUDE_RATE,
		.iLimit = 33.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 30.0,
	}
};

static CascadePidParam paramYaw = {
	.val = {
		.kp = 6.0,
		.ki = 1.0,
		.kd = 0.35,
		.rate = ATTITUDE_RATE,
		.iLimit = 360.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 15.0,
	},
	.rate = {
		.kp = 100.0,
		.ki = 20.0,
		.kd = 0.0,
		.rate = ATTITUDE_RATE,
		.iLimit = 33.0,
		.oLimit = 0.0,
		.enableDFilter = false,
		.cutoffFreq = 30.0,
	}
};
// static CascadePidParam paramRoll = {
// 	.val = {
// 		.kp = 6.0,
// 		.ki = 3.0,
// 		.kd = 0.0,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 20.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 15.0,
// 	},
// 	.rate = {
// 		.kp = 250.0,
// 		.ki = 500.0,
// 		.kd = 2.5,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 33.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 30.0,
// 	}
// };

// static CascadePidParam paramPitch = {
// 	.val = {
// 		.kp = 6.0,
// 		.ki = 3.0,
// 		.kd = 0.0,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 20.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 15.0,
// 	},
// 	.rate = {
// 		.kp = 250.0,
// 		.ki = 500.0,
// 		.kd = 2.5,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 33.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 30.0,
// 	}
// };

// static CascadePidParam paramYaw = {
// 	.val = {
// 		.kp = 6.0,
// 		.ki = 1.0,
// 		.kd = 0.35,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 360.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 15.0,
// 	},
// 	.rate = {
// 		.kp = 120.0,
// 		.ki = 17.0,
// 		.kd = 0.0,
// 		.rate = ATTITUDE_RATE,
// 		.iLimit = 167.0,
// 		.oLimit = 0.0,
// 		.enableDFilter = false,
// 		.cutoffFreq = 30.0,
// 	}
// };

void controllerPidAttitudeInit() {
	pidInit(&roll.val, &paramRoll.val);
	pidInit(&roll.rate, &paramRoll.rate);
	pidInit(&pitch.val, &paramPitch.val);
	pidInit(&pitch.rate, &paramPitch.rate);
	pidInit(&yaw.val, &paramYaw.val);
	pidInit(&yaw.rate, &paramYaw.rate);
}

void controllerPidAttitudeRateUpdate(rate_t measure, rate_t target, accel_t *output) {
	output->roll = pidUpdate(&roll.rate, measure.roll, target.roll, true);
	output->pitch = pidUpdate(&pitch.rate, measure.pitch, target.pitch, true);
	output->yaw = pidUpdate(&yaw.rate, measure.yaw, target.yaw, true);
}

void controllerPidAttitudeValUpdate(attitude_t measure, attitude_t target, rate_t *output) {
	output->roll = pidUpdate(&roll.val, measure.roll, target.roll, true);
	output->pitch = pidUpdate(&pitch.val, measure.pitch, target.pitch, true);
	pidSetError(&yaw.val, capAngle(target.yaw - measure.yaw));
	output->yaw = pidUpdate(&yaw.val, measure.yaw, target.yaw, false);
}

void controllerPidAttitudeValReset(uint8_t rpy) {
	if (rpy & PID_ROLL)
		pidReset(&roll.val);
	if (rpy & PID_PITCH)
		pidReset(&pitch.val);
	if (rpy & PID_YAW)
		pidReset(&yaw.val);
}

void controllerPidAttitudeRateReset(uint8_t rpy) {
	if (rpy & PID_ROLL)
		pidReset(&roll.rate);
	if (rpy & PID_PITCH)
		pidReset(&pitch.rate);
	if (rpy & PID_YAW)
		pidReset(&yaw.rate);
}

void controllerPidAttitudeResetAll() {
	controllerPidAttitudeValReset(PID_ROLL | PID_PITCH | PID_YAW);
	controllerPidAttitudeRateReset(PID_ROLL | PID_PITCH | PID_YAW);
}

// TODO: add log and param