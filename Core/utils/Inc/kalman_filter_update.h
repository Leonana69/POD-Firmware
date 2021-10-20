#ifndef __KALMAN_FILTER_UPDATE__
#define __KALMAN_FILTER_UPDATE__

#include "kalman_filter.h"
#include "stabilizer_types.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof);
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);
void kalmanCoreUpdateWithBaro(kalmanCoreData_t* this, float baroAsl, bool quadIsFlying);

#endif //__KALMAN_FILTER_UPDATE__
