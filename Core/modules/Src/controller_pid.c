#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller_pid.h"
#include "controller_pid_attitude.h"
#include "controller_pid_position.h"

#include "pid.h"
#include "cal.h"
#include "log.h"
#include "param.h"

static bool isInit = false;
static bool tiltCompensationEnabled = false;

static attitude_t attitudeTarget;
static rate_t rateTarget;
static accel_t output;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerPidInit(void) {
	if (isInit)
		return;
  controllerPidAttitudeInit();
	controllerPidPositionInit();
	isInit = true;
}

bool controllerPidTest(void) {
  return isInit;
}

void controllerPidUpdate(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick) {
	// TODO: move this after postition control
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeTarget.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else
      attitudeTarget.yaw = setpoint->attitude.yaw;

    attitudeTarget.yaw = capAngle(attitudeTarget.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
		controllerPidPositionUpdate(&actuatorThrust, &attitudeTarget, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeTarget.roll = setpoint->attitude.roll;
      attitudeTarget.pitch = setpoint->attitude.pitch;
    }

		controllerPidAttitudeValUpdate(state->attitude, attitudeTarget, &rateTarget);
    // For roll and pitch, if velocity mode, overwrite rateTarget with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateTarget.roll = setpoint->attitudeRate.roll;
      controllerPidAttitudeValReset(PID_ROLL);
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateTarget.pitch = setpoint->attitudeRate.pitch;
      controllerPidAttitudeValReset(PID_PITCH);
    }

    // TODO: Investigate possibility to subtract gyro drift.
		rate_t measure = {
			.roll = sensors->gyro.x,
			.pitch = -sensors->gyro.y,
			.yaw = sensors->gyro.z,
		};
		controllerPidAttitudeRateUpdate(measure, rateTarget, &output);

		control->roll = capValueInt16(output.roll);
		control->pitch = capValueInt16(output.pitch);
		control->yaw = -capValueInt16(output.yaw);
  }

  if (tiltCompensationEnabled) {
		// TODO: compensate gravity
    control->thrust = actuatorThrust;// / sensfusion6GetInvThrustCompensationForTilt();
  } else
    control->thrust = actuatorThrust;

  if (control->thrust == 0) {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;
		controllerPidAttitudeResetAll();
		// TODO: check if we need to reset the filter
		controllerPidPositionResetAll(false);
    // Reset the calculated YAW angle for rate control
    attitudeTarget.yaw = state->attitude.yaw;
  }

	cmd_thrust = control->thrust;
	cmd_roll = control->roll;
	cmd_pitch = control->pitch;
	cmd_yaw = control->yaw;
	r_roll = radians(sensors->gyro.x);
	r_pitch = -radians(sensors->gyro.y);
	r_yaw = radians(sensors->gyro.z);
	accelz = sensors->accel.z;
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Controller output
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro measurements in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Target roll, pitch, and yaw
 */
LOG_ADD(LOG_FLOAT, roll, &attitudeTarget.roll)
LOG_ADD(LOG_FLOAT, pitch, &attitudeTarget.pitch)
LOG_ADD(LOG_FLOAT, yaw, &attitudeTarget.yaw)
/**
 * @brief Target roll, pitch, and yaw rates
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateTarget.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateTarget.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateTarget.yaw)
LOG_GROUP_STOP(controller)


/**
 * Controller parameters
 */
PARAM_GROUP_START(controller)
/**
 * @brief Nonzero for tilt compensation enabled (default: 0)
 */
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
