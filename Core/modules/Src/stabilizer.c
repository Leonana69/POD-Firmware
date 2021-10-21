/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "cal.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
// #include "crtp_localization_service.h"
#include "controller.h"
#include "estimator.h"
#include "power_distribution.h"
#include "self_test.h"
#include "supervisor.h"

// #include "usddeck.h"
// #include "statsCnt.h"
#include "static_mem.h"
// #include "rateSupervisor.h"

static bool isInit = false;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

// static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
// static rateSupervisor_t rateSupervisorContext;
// static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask();

static void calcSensorToOutputLatency(const sensorData_t *sensorData) {
  uint64_t outTimestamp = osKernelGetTickCount() * 1000 / osKernelGetTickFreq();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState() {
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * GRAVITY_EARTH * 1000.0f;
  stateCompressed.ay = state.acc.y * GRAVITY_EARTH * 1000.0f;
	// TODO: check this
  stateCompressed.az = (state.acc.z + 1) * GRAVITY_EARTH * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w
	};
  stateCompressed.quat = quaternionCompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint() {
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit() {
  if (isInit)
    return;
  
  estimatorInit();
  controllerInit();
  sensorsInit();
  powerDistributionInit();
  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);
  isInit = true;
}

bool stabilizerTest(void) {
  bool pass = true;

  pass &= sensorsTest();
  pass &= estimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  return pass;
}

static void checkEmergencyStopTimeout() {
	emergencyStop = (emergencyStopTimeout-- <= 0);
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask() {
  /*! Wait for the system to be fully started */
  systemWaitStart();

  uint32_t tick = 1;
  uint32_t lastWakeTime = osKernelGetTickCount();
  DEBUG_PRINT("Wait for sensor calibration...\n");
  while (!sensorsAreCalibrated())
    osDelay(10);

  // rateSupervisorInit(&rateSupervisorContext, osKernelGetTickCount(), 1000, 997, 1003, 1);

  DEBUG_PRINT("Ready to fly.\n");
  while (1) {
    /*! The sensor should unlock at 1kHz */
    sensorsWaitDataReady();
    sensorsAcquire(&sensorData);
    /*! Update the drone flight state */
    supervisorUpdate(&sensorData);

    // TODO: check selfTest
    if (1 || selfTestPassed()) {
      estimatorUpdate(&state, tick);
      compressState();

      commanderGetSetpoint(&setpoint, &state);
      compressSetpoint();

      // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      controllerUpdate(&control, &setpoint, &sensorData, &state, tick);

      // checkEmergencyStopTimeout();

      if (emergencyStop)
        powerStop();
      else
        powerDistributionUpdate(&control);

      // Log data to uSD card if configured
      // if (   usddeckLoggingEnabled()
      //     && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
      //     && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
      //   usddeckTriggerLogging();
      // }

      calcSensorToOutputLatency(&sensorData);
      tick++;
      // TODO: check the rate
      // STATS_CNT_RATE_EVENT(&stabilizerRate);
      // if (!rateSupervisorValidate(&rateSupervisorContext, osKernelGetTickCount())) {
      //   if (!rateWarningDisplayed) {
      //     DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      //     rateWarningDisplayed = true;
      //   }
      // }
    } else
      selfTestRun(&sensorData);
      
  }
}

void stabilizerSetEmergencyStop() {
  emergencyStop = true;
}

void stabilizerResetEmergencyStop() {
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout) {
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/**
 * Parameters to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)
PARAM_GROUP_STOP(stabilizer)


/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
LOG_GROUP_START(ctrltarget)

/**
 * @brief Desired position [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)

/**
 * @brief Desired velocity [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)
LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)
LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)

/**
 * @brief Desired acceleration [m / s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)

/**
 * @brief Desired attitude [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)

LOG_GROUP_STOP(ctrltarget)

/**
 * Log group for the current controller target, compressed format.
 * This flavour of the controller target logs are defined with types
 * that use less space and makes it possible to add more logs to a
 * log configuration.
 *
 * Note: all members may not be updated depending on how the system is used
 */

LOG_GROUP_START(ctrltargetZ)
/**
 * @brief Desired position [mm]
 */
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

/**
 * @brief Desired velocity [mm / s]
 */
LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx)
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

/**
 * @brief Desired acceleration [mm / s^2]
 */
LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax)
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)

LOG_GROUP_STOP(ctrltargetZ)

/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(acc)
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.accel.x)
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.accel.y)
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.accel.z)
LOG_GROUP_STOP(acc)

/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(gyro)
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(baro)

/**
 * @brief Altitude above Sea Level [m]
 */
LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)

/**
 * @brief Temperature [degrees Celsius]
 */
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)

/**
 * @brief Air preassure [mbar]
 */
LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
LOG_ADD(LOG_INT16, yaw, &control.yaw)
LOG_GROUP_STOP(controller)

/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimate)
/**
 * @brief The estimated position of the platform in the global reference frame [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &state.position.x)
LOG_ADD_CORE(LOG_FLOAT, y, &state.position.y)
LOG_ADD_CORE(LOG_FLOAT, z, &state.position.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame [m / s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD_CORE(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD_CORE(LOG_FLOAT, vz, &state.velocity.z)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame [m / s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &state.acc.x)
LOG_ADD_CORE(LOG_FLOAT, ay, &state.acc.y)
LOG_ADD_CORE(LOG_FLOAT, az, &state.acc.z)

/**
 * @brief Attitude, roll, pitch and yaw angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD_CORE(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD_CORE(LOG_FLOAT, yaw, &state.attitude.yaw)

/**
 * @brief Attitude as a quaternion
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
LOG_ADD_CORE(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
LOG_ADD_CORE(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
LOG_ADD_CORE(LOG_FLOAT, qw, &state.attitudeQuaternion.w)

LOG_GROUP_STOP(stateEstimate)

/**
 * Log group for the state estimator, compressed format. This flavour of the
 * estimator logs are defined with types that use less space and makes it possible to
 * add more logs to a log configuration.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimateZ)

/**
 * @brief The position of the Crazyflie in the global reference frame [mm]
 */
LOG_ADD(LOG_INT16, x, &stateCompressed.x)
LOG_ADD(LOG_INT16, y, &stateCompressed.y)
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame [mm / s]
 */

// @brief The velocity of the Crazyflie in the global reference frame [mm / s]
LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame [mm / s^2]
 */
LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

/**
 * @brief Attitude as a compressed quaternion, see cal.c for details
 */
LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)

/**
 * @brief Angular velocity of roll, pitch and yaw [milliradians / s]
 */
LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)

LOG_GROUP_STOP(stateEstimateZ)
