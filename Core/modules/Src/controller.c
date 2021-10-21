#define DEBUG_MODULE "CTRL"
#include "debug.h"
#include "config.h"
#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
// #include "controller_mellinger.h"
// #include "controller_indi.h"

static ControllerType currentController = CONTROLLER_TYPE;

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} Controller;

static Controller controllerFunctions[] = {
  { .init = controllerPidInit, .test = controllerPidTest, .update = controllerPidUpdate, .name = "PID" },
  // { .init = controllerMellingerInit, .test = controllerMellingerTest, .update = controllerMellinger, .name = "Mellinger" },
  // { .init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI" },
};

void controllerInit() {
  controllerFunctions[currentController].init();
  DEBUG_PRINT("Using %s (%d) controller.\n", controllerGetName(), currentController);
}

ControllerType controllerGetType() {
  return currentController;
}

bool controllerTest() {
  return controllerFunctions[currentController].test();
}

void controllerUpdate(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, tick);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
