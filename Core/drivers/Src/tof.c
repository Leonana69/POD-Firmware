#define DEBUG_MODULE "TOF"

#include "tof.h"

#include "debug.h"
#include "config.h"
#include "vl53l1_platform.h"

static bool isInit = false;

void tofInit() {
	if (isInit)
		return;

	isInit = true;
}

bool tofTest() {
	return isInit;
}

/*! vl53l1 platform functions */