#ifndef __SELF_TEST_H__
#define __SELF_TEST_H__

#include "sensors.h"
#include <stdbool.h>

bool selfTestPassed();
void selfTestRun(sensorData_t *sensors);

#endif //__SELF_TEST_H__
