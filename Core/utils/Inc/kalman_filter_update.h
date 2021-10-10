#ifndef __KALMAN_FILTER_UPDATE__
#define __KALMAN_FILTER_UPDATE__

#include "kalman_filter.h"
#include "stabilizer_types.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof);

#endif //__KALMAN_FILTER_UPDATE__