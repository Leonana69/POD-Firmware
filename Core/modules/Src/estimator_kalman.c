/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */

#include "estimator.h"
#include "estimator_kalman.h"
#include "kalman_filter.h"
#include "kalman_filter_update.h"

// #include "FreeRTOS.h"
// #include "queue.h"
// #include "task.h"
// #include "semphr.h"
// #include "sensors.h"
#include "static_mem.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "config.h"
// #include "physicalConstants.h"
#include "supervisor.h"

// #include "statsCnt.h"
// #include "rateSupervisor.h"

// Measurement models
// #include "mm_distance.h"
// #include "mm_absolute_height.h"
// #include "mm_position.h"
// #include "mm_pose.h"
// #include "mm_tdoa.h"
// #include "mm_flow.h"
// #include "mm_tof.h"
// #include "mm_yaw_error.h"
// #include "mm_sweep_angles.h"

// #include "mm_tdoa_robust.h"
// #include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTKALMAN"



// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
// TODO: change kalman to a function
STATIC_MEM_SEMAPHORE_ALLOC(runKalman);

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
STATIC_MEM_MUTEX_ALLOC(dataMutex);

/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3f accAccumulator;
static Axis3f gyroAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static Axis3f accLatest;
static Axis3f gyroLatest;
static bool quadIsFlying = false;


// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.

// Statistics
// #define ONE_SECOND 1000
// static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

// static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME 2000
static uint32_t warningBlockTime = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask();
static void estimatorKalmanReset();
static bool predictStateForward(uint32_t osTick, float dt);
static bool updateQueuedMeasurements(const uint32_t tick);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, 3 * configMINIMAL_STACK_SIZE);

// --------------------------------------------------

static void kalmanTask() {
  systemWaitStart();

	float dt;
  bool doneUpdate;
  uint32_t lastPrediction = osKernelGetTickCount();
  uint32_t nextPrediction = osKernelGetTickCount();
  uint32_t lastPNUpdate = osKernelGetTickCount();

  // rateSupervisorInit(&rateSupervisorContext, osKernelGetTickCount(), ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);

  while (1) {
    osSemaphoreAcquire(runKalman, osWaitForever);

    // If the client triggers an estimator reset via parameter update
    if (coreData.resetEstimation) {
      estimatorKalmanReset();
      paramSetInt(paramGetVarId("kalman", "resetEstimation"), 0);
    }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    doneUpdate = false;
    uint32_t osTick = osKernelGetTickCount(); // would be nice if this had a precision higher than 1ms...

  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif

    // Run the system dynamics to predict the state forward.
    if (osTick >= nextPrediction) { // update at the PREDICT_RATE
      dt = (osTick - lastPrediction) / (float)osKernelGetTickFreq();
      if (predictStateForward(osTick, dt)) {
        lastPrediction = osTick;
        doneUpdate = true;
				// TODO: add this
        // STATS_CNT_RATE_EVENT(&predictionCounter);
      }
      nextPrediction = osTick + osKernelGetTickFreq() / PREDICT_RATE;

      // if (!rateSupervisorValidate(&rateSupervisorContext, T2M(osTick))) {
      //   DEBUG_PRINT("WARNING: Kalman prediction rate low (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      // }
    }

    /**
     * Add process noise every loop, rather than every prediction
     */
		dt = (osTick - lastPNUpdate) / (float)osKernelGetTickFreq();
		if (dt > 0.0f) {
			kalmanCoreAddProcessNoise(&coreData, dt);
			lastPNUpdate = osTick;
		}

		if (updateQueuedMeasurements(osTick)) {
			doneUpdate = true;
		}

    /**
     * If an update has been made, the state is finalized:
     * - the attitude error is moved into the body attitude quaternion,
     * - the body attitude is converted into a rotation matrix for the next prediction, and
     * - correctness of the covariance matrix is ensured
     */

    if (doneUpdate) {
      kalmanCoreFinalize(&coreData, osTick);
      // STATS_CNT_RATE_EVENT(&finalizeCounter);
      if (!supervisorKalmanIsStateWithinBounds(&coreData)) {
        coreData.resetEstimation = true;

        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    osMutexAcquire(dataMutex, osWaitForever);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest, osTick);
    osMutexRelease(dataMutex);
		// TODO: add this
    // STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

void estimatorKalmanUpdate(state_t *state, const uint32_t tick) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  osMutexAcquire(dataMutex, osWaitForever);
  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  osMutexRelease(dataMutex);
  osSemaphoreRelease(runKalman);
}

static bool predictStateForward(uint32_t osTick, float dt) {
  if (gyroAccumulatorCount == 0 || accAccumulatorCount == 0)
    return false;
  // gyro is in deg/sec but the estimator requires rad/sec
  Axis3f gyroAverage;
  gyroAverage.x = radians(gyroAccumulator.x) / gyroAccumulatorCount;
  gyroAverage.y = radians(gyroAccumulator.y) / gyroAccumulatorCount;
  gyroAverage.z = radians(gyroAccumulator.z) / gyroAccumulatorCount;

  // accelerometer is in Gs but the estimator requires ms^-2
  Axis3f accAverage;
  accAverage.x = accAccumulator.x * GAS / accAccumulatorCount;
  accAverage.y = accAccumulator.y * GAS / accAccumulatorCount;
  accAverage.z = accAccumulator.z * GAS / accAccumulatorCount;

  // reset for next call
  accAccumulator = (Axis3f){ .axis = { 0 } };
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){ .axis = { 0 } };
  gyroAccumulatorCount = 0;

  quadIsFlying = supervisorIsFlying();
  kalmanCorePredict(&coreData, &accAverage, &gyroAverage, dt, quadIsFlying);
  return true;
}


static bool updateQueuedMeasurements(const uint32_t tick) {
  bool doneUpdate = false;
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      // case MeasurementTypeTDOA:
      //   if (robustTdoa){
      //     // robust KF update with TDOA measurements
      //     kalmanCoreRobustUpdateWithTDOA(&coreData, &m.data.tdoa);
      //   } else {
      //     // standard KF update
      //     kalmanCoreUpdateWithTDOA(&coreData, &m.data.tdoa);
      //   }
      //   doneUpdate = true;
      //   break;
      // case MeasurementTypePosition:
      //   kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
      //   doneUpdate = true;
      //   break;
      // case MeasurementTypePose:
      //   kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
      //   doneUpdate = true;
      //   break;
      // case MeasurementTypeDistance:
      //   if (robustTwr){
      //       // robust KF update with UWB TWR measurements
      //       kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
      //   } else {
      //       // standard KF update
      //       kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
      //   }
      //   doneUpdate = true;
      //   break;

      case MeasurementTypeTOF:
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        doneUpdate = true;
        break;

      // case MeasurementTypeAbsoluteHeight:
      //   kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
      //   doneUpdate = true;
      //   break;

      // TODO: enable Flow
      // case MeasurementTypeFlow:
      //   kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
      //   doneUpdate = true;
      //   break;

      // case MeasurementTypeYawError:
      //   kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
      //   doneUpdate = true;
      //   break;

      case MeasurementTypeGyroscope:
        gyroAccumulator.x += m.data.gyroscope.gyro.x;
        gyroAccumulator.y += m.data.gyroscope.gyro.y;
        gyroAccumulator.z += m.data.gyroscope.gyro.z;
        gyroLatest = m.data.gyroscope.gyro;
        gyroAccumulatorCount++;
        break;
      case MeasurementTypeAcceleration:
        accAccumulator.x += m.data.acceleration.acc.x;
        accAccumulator.y += m.data.acceleration.acc.y;
        accAccumulator.z += m.data.acceleration.acc.z;
        accLatest = m.data.acceleration.acc;
        accAccumulatorCount++;
        break;
        // TODO: add baro
      // case MeasurementTypeBarometer:
      //   if (useBaroUpdate) {
      //     kalmanCoreUpdateWithBaro(&coreData, m.data.barometer.baro.asl, quadIsFlying);
      //     doneUpdate = true;
      //   }
      //   break;
      default:
        break;
    }
  }

  return doneUpdate;
}

// Called when this estimator is activated
void estimatorKalmanInit() {
	if (isInit)
		return;
  estimatorKalmanReset();
	STATIC_SEMAPHORE_CREATE(runKalman, 1, 0);
	STATIC_MUTEX_CREATE(dataMutex);
  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);
	isInit = true;
}

void estimatorKalmanReset() {
  accAccumulator = (Axis3f){ .axis = { 0 } };
  gyroAccumulator = (Axis3f){ .axis = { 0 } };

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  kalmanCoreInit(&coreData);
}

bool estimatorKalmanTest() {
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t *pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float *rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9 * sizeof(float));
}

// TODO: check the list
/**
 * Variables and results from the Extended Kalman Filter
 */
LOG_GROUP_START(kalman)
  /**
	 * @brief State position in the global frame x, y, z
	 * 
	 *   Note: This is similar to stateEstimate.x, y, z
	 */
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  /**
	 * @brief State position in the global frame PX, PY, PZ
	 * 
	 *  Note: This is similar to stateEstimate.x, y, z
	 */
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  /**
   * @brief State attitude error roll, pitch, yaw
   */
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  /**
  * @brief Covariance matrix position x, y, z
  */
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  /**
  * @brief Covariance matrix velocity x, y, z
  */
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  /**
  * @brief Covariance matrix attitude error roll, pitch and yaw
  */
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  /**
  * @brief Estimated Attitude quarternion w, x, y, z
  */
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
  /**
  * @brief Statistics rate of update step
  */
 // TODO: add cnt
  // STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  /**
  * @brief Statistics rate of prediction step
  */
  // STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  /**
  * @brief Statistics rate full estimation step
  */
  // STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
LOG_GROUP_STOP(kalman)

/**
 * Tuning parameters for the Extended Kalman Filter (EKF)
 *     estimator
 */
PARAM_GROUP_START(kalman)
/**
 * @brief Reset the kalman estimator
 */
  PARAM_ADD_CORE(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
/**
 * @brief Nonzero to use robust TDOA method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTdoa, &robustTdoa)
/**
 * @brief Nonzero to use robust TWR method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTwr, &robustTwr)
PARAM_GROUP_STOP(kalman)
