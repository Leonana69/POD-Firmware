#include <stdbool.h>
#include "static_mem.h"
#include "debug.h"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "log.h"
// #include "statsCnt.h"
// #include "eventtrigger.h"
#include "cal.h"
#include "config.h"

#define DEBUG_MODULE "ESTI"

static EstimatorType currentEstimator = ESTIMATOR_TYPE;

#define MEASUREMENTS_QUEUE_SIZE (20)

STATIC_MEM_QUEUE_ALLOC(measurementsQueue, MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t));

// Statistics
// #define ONE_SECOND 1000
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

// events
// EVENTTRIGGER(estTDOA, uint8, idA, uint8, idB, float, distanceDiff)
// EVENTTRIGGER(estPosition, uint8, source)
// EVENTTRIGGER(estPose)
// EVENTTRIGGER(estDistance, uint8, id, float, distance)
// EVENTTRIGGER(estTOF)
// EVENTTRIGGER(estAbsoluteHeight)
// EVENTTRIGGER(estFlow)
// EVENTTRIGGER(estYawError, float, yawError)
// EVENTTRIGGER(estSweepAngle, uint8, sensorId, uint8, basestationId, uint8, sweepId, float, t, float, sweepAngle)
// EVENTTRIGGER(estGyroscope)
// EVENTTRIGGER(estAcceleration)
// EVENTTRIGGER(estBarometer)

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(state_t *state, const uint32_t tick);
  const char* name;
} Estimator;

#define NOT_IMPLEMENTED ((void*)0)

static Estimator estimatorFunctions[] = {
	{
		.init = estimatorKalmanInit,
		.test = estimatorKalmanTest,
		.update = estimatorKalmanUpdate,
		.name = "KALMAN",
	},
#ifdef OOT_ESTIMATOR
	{
		.init = estimatorOutOfTreeInit,
		.deinit = NOT_IMPLEMENTED,
		.test = estimatorOutOfTreeTest,
		.update = estimatorOutOfTree,
		.name = "OutOfTree",
	},
#endif
};

void estimatorInit() {
	estimatorFunctions[currentEstimator].init();
	DEBUG_PRINT("Using %s (%d) estimator.\n", estimatorGetName(), currentEstimator);

  STATIC_MEM_QUEUE_CREATE(measurementsQueue);
}

bool estimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

void estimatorUpdate(state_t *state, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, tick);
}

EstimatorType estimatorGetType(void) {
  return currentEstimator;
}

const char* estimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}

void estimatorEnqueue(const measurement_t *measurement) {
  if (!measurementsQueue)
    return;
  
	osMessageQueuePut(measurementsQueue, measurement, 0, 0);

  // if (result == pdTRUE) {
  //   STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
  // } else {
  //   STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
  // }

	// TODO: remove these

  // events
  // switch (measurement->type) {
  //   case MeasurementTypeTDOA:
  //     eventTrigger_estTDOA_payload.idA = measurement->data.tdoa.anchorIds[0];
  //     eventTrigger_estTDOA_payload.idB = measurement->data.tdoa.anchorIds[1];
  //     eventTrigger_estTDOA_payload.distanceDiff = measurement->data.tdoa.distanceDiff;
  //     eventTrigger(&eventTrigger_estTDOA);
  //     break;
  //   case MeasurementTypePosition:
  //     // for additional data, see locSrv.{x,y,z} and lighthouse.{x,y,z}
  //     eventTrigger_estPosition_payload.source = measurement->data.position.source;
  //     eventTrigger(&eventTrigger_estPosition);
  //     break;
  //   case MeasurementTypePose:
  //     // no payload needed, see locSrv.{x,y,z,qx,qy,qz,qw}
  //     eventTrigger(&eventTrigger_estPose);
  //     break;
  //   case MeasurementTypeDistance:
  //     eventTrigger_estDistance_payload.id = measurement->data.distance.anchorId;
  //     eventTrigger_estDistance_payload.distance = measurement->data.distance.distance;
  //     eventTrigger(&eventTrigger_estDistance);
  //     break;
  //   case MeasurementTypeTOF:
  //     // no payload needed, see range.zrange
  //     eventTrigger(&eventTrigger_estTOF);
  //     break;
  //   case MeasurementTypeAbsoluteHeight:
  //     // no payload needed, see LPS_2D_POSITION_HEIGHT
  //     eventTrigger(&eventTrigger_estAbsoluteHeight);
  //     break;
  //   case MeasurementTypeFlow:
  //     // no payload needed, see motion.{deltaX,deltaY}
  //     eventTrigger(&eventTrigger_estFlow);
  //     break;
  //   case MeasurementTypeYawError:
  //     eventTrigger_estYawError_payload.yawError = measurement->data.yawError.yawError;
  //     eventTrigger(&eventTrigger_estYawError);
  //     break;
  //   case MeasurementTypeSweepAngle:
  //     eventTrigger_estSweepAngle_payload.sensorId = measurement->data.sweepAngle.sensorId;
  //     eventTrigger_estSweepAngle_payload.basestationId = measurement->data.sweepAngle.basestationId;
  //     eventTrigger_estSweepAngle_payload.sweepId = measurement->data.sweepAngle.sweepId;
  //     eventTrigger_estSweepAngle_payload.t = measurement->data.sweepAngle.t;
  //     eventTrigger_estSweepAngle_payload.sweepAngle = measurement->data.sweepAngle.measuredSweepAngle;
  //     eventTrigger(&eventTrigger_estSweepAngle);
  //     break;
  //   case MeasurementTypeGyroscope:
  //     // no payload needed, see gyro.{x,y,z}
  //     eventTrigger(&eventTrigger_estGyroscope);
  //     break;
  //   case MeasurementTypeAcceleration:
  //     // no payload needed, see acc.{x,y,z}
  //     eventTrigger(&eventTrigger_estAcceleration);
  //     break;
  //   case MeasurementTypeBarometer:
  //     // no payload needed, see baro.asl
  //     eventTrigger(&eventTrigger_estBarometer);
  //     break;
  //   default:
  //     break;
  // }
}

bool estimatorDequeue(measurement_t *measurement) {
  return osOK == osMessageQueueGet(measurementsQueue, measurement, NULL, 0);
}
