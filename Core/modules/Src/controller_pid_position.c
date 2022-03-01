#include "pid.h"
#include "stabilizer_types.h"
#include "cal.h"
#include "controller_pid_position.h"
#include "debug.h"

static CascadePidObject cpidX;
static CascadePidObject cpidY;
static CascadePidObject cpidZ;

// TODO: add IMPROVED_BARO_Z_HOLD

static CascadePidParam paramX = {
	.val = {
		.kp = 2.0,
		.ki = 0.5,
		.kd = 0.0,
		.rate = POSITION_RATE,
		.iLimit = 1.0,
		.oLimit = 0,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	},
	.rate = {
		.kp = 25.0,
		.ki = 2.0,
		.kd = 0.0,
		.rate = POSITION_RATE,
		.iLimit = 25.0,
		.oLimit = 20.0,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	}
};

static CascadePidParam paramY = {
	.val = {
		.kp = 2.0,
		.ki = 0.5,
		.kd = 0.0,
		.rate = POSITION_RATE,
		.iLimit = 1.0,
		.oLimit = 0,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	},
	.rate = {
		.kp = 25.0,
		.ki = 2.0,
		.kd = 0.0,
		.rate = POSITION_RATE,
		.iLimit = 25.0,
		.oLimit = 20.0,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	}
};

static CascadePidParam paramZ = {
	.val = {
		.kp = 2.0,
		.ki = 0.5,
		.kd = 0.0,
		.rate = POSITION_RATE,
		.iLimit = 1.0,
		.oLimit = 0.0,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	},
	.rate = {
		.kp = 25.0,
		.ki = 15.0,
		.kd = 1.0,
		.rate = POSITION_RATE,
		.iLimit = 30.0,
		.oLimit = INT16_MAX / THRUST_SCALE,
		.enableDFilter = true,
		.cutoffFreq = 20.0,
	}
};

void controllerPidPositionInit() {
	pidInit(&cpidX.val, &paramX.val);
	pidInit(&cpidX.rate, &paramX.rate);
	pidInit(&cpidY.val, &paramY.val);
	pidInit(&cpidY.rate, &paramY.rate);
	pidInit(&cpidZ.val, &paramZ.val);
	pidInit(&cpidZ.rate, &paramZ.rate);
}

void controllerPidPositionUpdate(float* thrust, attitude_t *attitude,
																 setpoint_t *setpoint, const state_t *state) {
	float cosYaw = cosf(radians(state->attitude.yaw));
	float sinYaw = sinf(radians(state->attitude.yaw));

	float vx = setpoint->velocity.x;
	float vy = setpoint->velocity.y;

	if (setpoint->mode.x == modeAbs) {
		setpoint->velocity.x = pidUpdate(&cpidX.val, state->position.x, setpoint->position.x, true);
	} else if (setpoint->velocity_body)
		setpoint->velocity.x = vx * cosYaw - vy * sinYaw;

	if (setpoint->mode.y == modeAbs) {
		setpoint->velocity.y = pidUpdate(&cpidY.val, state->position.y, setpoint->position.y, true);
	} else if (setpoint->velocity_body)
		setpoint->velocity.y = vy * cosYaw + vx * sinYaw;

	if (setpoint->mode.z == modeAbs) {
		setpoint->velocity.z = pidUpdate(&cpidZ.val, state->position.z, setpoint->position.z, true);
	}

	float accX = pidUpdate(&cpidX.rate, state->velocity.x, setpoint->velocity.x, true);
	float accY = pidUpdate(&cpidY.rate, state->velocity.y, setpoint->velocity.y, true);
	// pitch+ <-> x-, roll+ <-> y-
	// attitude->pitch = -fConstrain(accX * cosYaw + accY * sinYaw, -20.0, 20.0);
	// attitude->roll = -fConstrain(accY * cosYaw - accX * sinYaw, -20.0, 20.0);
	attitude->pitch = -(accX * cosYaw + accY * sinYaw);
	attitude->roll = -(accY * cosYaw - accX * sinYaw);

	// static int cnt = 0;
	// if (cnt++ == 50) {
	// 	cnt = 0;
	// 	DEBUG_PRINT_CONSOLE("%.2f %.2f %.2f %.2f\n", state->attitude.roll, 
	// 	state->attitude.pitch, state->position.x, state->position.y);
	// }

	float accZ = pidUpdate(&cpidZ.rate, state->velocity.z, setpoint->velocity.z, true);

	*thrust = accZ * THRUST_SCALE + BASE_THRUST;
	if (*thrust < MIN_THRUST)
		*thrust = MIN_THRUST;
}

void controllerPidPositionValReset(uint8_t xyz) {
	if (xyz & PID_X)
		pidReset(&cpidX.val);
	if (xyz & PID_Y)
		pidReset(&cpidY.val);
	if (xyz & PID_Z)
		pidReset(&cpidZ.val);
}

void controllerPidPositionRateReset(uint8_t xyz) {
	if (xyz & PID_X)
		pidReset(&cpidX.rate);
	if (xyz & PID_Y)
		pidReset(&cpidY.rate);
	if (xyz & PID_Z)
		pidReset(&cpidZ.rate);
}

void controllerPidPositionResetAll(bool resetFilter) {
	controllerPidPositionValReset(PID_X | PID_Y | PID_Z);
	controllerPidPositionRateReset(PID_X | PID_Y | PID_Z);
	if (resetFilter) {
		filterReset(&cpidX.val, paramX.val.rate, paramX.val.cutoffFreq, paramX.val.enableDFilter);
		filterReset(&cpidX.rate, paramX.rate.rate, paramX.rate.cutoffFreq, paramX.rate.enableDFilter);
		filterReset(&cpidY.val, paramY.val.rate, paramY.val.cutoffFreq, paramY.val.enableDFilter);
		filterReset(&cpidY.rate, paramY.rate.rate, paramY.rate.cutoffFreq, paramY.rate.enableDFilter);
		filterReset(&cpidZ.val, paramZ.val.rate, paramZ.val.cutoffFreq, paramZ.val.enableDFilter);
		filterReset(&cpidZ.rate, paramZ.rate.rate, paramZ.rate.cutoffFreq, paramZ.rate.enableDFilter);
	}
}

// TODO: add log and param