/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
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
 */

/*
 * ledseq.c - LED sequence handler
 */

#include <stdbool.h>
#include <string.h>

#include "ledseq.h"
#include "static_mem.h"

#include "led.h"
#include "debug.h"

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500

/* Led sequences */
ledseqStep_t seq_lowbat_def[] = {
  { true, LEDSEQ_WAITMS(1000) },
  {    0, LEDSEQ_LOOP },
};

ledseqContext_t seq_lowbat = {
  .sequence = seq_lowbat_def,
  .led = LOWBAT_LED,
};

#define NO_CONTEXT 0
ledseqContext_t* sequences = NO_CONTEXT;

ledseqStep_t seq_calibrated_def[] = {
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(450) },
  {     0, LEDSEQ_STOP },
};

ledseqContext_t seq_calibrated = {
  .sequence = seq_calibrated_def,
  .led = SYS_LED,
};

ledseqStep_t seq_alive_def[] = {
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(1950) },
  {     0, LEDSEQ_LOOP },
};

ledseqContext_t seq_alive = {
  .sequence = seq_alive_def,
  .led = SYS_LED,
};

ledseqStep_t seq_linkup_def[] = {
  {  true, LEDSEQ_WAITMS(1) },
  { false, LEDSEQ_WAITMS(0) },
  {     0, LEDSEQ_STOP },
};

ledseqContext_t seq_linkUp = {
  .sequence = seq_linkup_def,
  .led = LINK_LED,
};

ledseqContext_t seq_linkDown = {
  .sequence = seq_linkup_def,
  .led = LINK_DOWN_LED,
};

ledseqStep_t seq_charged_def[] = {
  { true, LEDSEQ_WAITMS(1000) },
  {    0, LEDSEQ_LOOP },
};

ledseqContext_t seq_charged = {
  .sequence = seq_charged_def,
  .led = CHG_LED,
};

ledseqStep_t seq_charging_def[] = {
  {  true, LEDSEQ_WAITMS(200) },
  { false, LEDSEQ_WAITMS(800) },
  {     0, LEDSEQ_LOOP },
};

ledseqContext_t seq_charging = {
  .sequence = seq_charging_def,
  .led = CHG_LED,
};

ledseqStep_t seq_testPassed_def[] = {
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  {  true, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_WAITMS(50) },
  { false, LEDSEQ_STOP },
};

ledseqContext_t seq_testPassed = {
  .sequence = seq_testPassed_def,
  .led = LINK_LED,
};

ledseqContext_t seq_testFailed = {
  .sequence = seq_testPassed_def,
  .led = SYS_LED,
};

struct ledseqCmd_s {
  enum { run, stop } command;
  ledseqContext_t *sequence;
};

/* Led sequence handling machine implementation */
static void runLedseq(void *name);
static void updateActive(led_t led);

NO_DMA_CCM_SAFE_ZERO_INIT static ledseqContext_t* activeSeq[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static osTimerId_t timer[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t timerBuffer[LED_NUM];
static char *ledseqName[] = {
	"leds0",
	"leds1",
	"leds2",
	"leds3",
	"leds4",
};

STATIC_MEM_MUTEX_ALLOC(ledseqMutex);

static osMessageQueueId_t ledseqCmdQueue;

static bool isInit = false;
static bool ledseqEnabled = false;

static void lesdeqCmdTask();

void ledseqInit() {
  if (isInit)
    return;

  ledInit();

  /* Led sequence priority */
  ledseqRegisterSequence(&seq_testPassed);
  ledseqRegisterSequence(&seq_testFailed);
  ledseqRegisterSequence(&seq_lowbat);
  ledseqRegisterSequence(&seq_charged);
  ledseqRegisterSequence(&seq_charging);
  ledseqRegisterSequence(&seq_calibrated);
  ledseqRegisterSequence(&seq_alive);
  ledseqRegisterSequence(&seq_linkUp);
  ledseqRegisterSequence(&seq_linkDown);
	
  //Init the soft timers that runs the led sequences for each leds
  for (int i = 0; i < LED_NUM; i++) {
		osTimerAttr_t timAttr = {
			.name = ledseqName[i],
			.cb_mem = timerBuffer + i,
			.cb_size = sizeof(timerBuffer[i])
		};
		timer[i] = osTimerNew(runLedseq, osTimerOnce, ledseqName[i], &timAttr);
  }
	

  STATIC_MUTEX_CREATE(ledseqMutex);
  ledseqCmdQueue = osMessageQueueNew(10, sizeof(struct ledseqCmd_s), NULL);

	osThreadAttr_t thrAttr = {
		.name = LEDSEQCMD_TASK_NAME,
		.priority = LEDSEQCMD_TASK_PRI,
	};
	osThreadNew(lesdeqCmdTask, NULL, &thrAttr);
  
	ledseqEnable(true);
  isInit = true;
}

static void lesdeqCmdTask() {
  struct ledseqCmd_s command;
  while (1) {
    osMessageQueueGet(ledseqCmdQueue, &command, NULL, osWaitForever);
    switch(command.command) {
      case run:
        ledseqRunBlocking(command.sequence);
        break;
      case stop:
        ledseqStopBlocking(command.sequence);
        break;
    }
  }
}

bool ledseqTest(void) {
  bool status = isInit & ledTest();
  #ifdef TURN_OFF_LEDS
  ledseqEnable(false);
  ledSet(LED_BLUE_L, 0);
  #else
  ledseqEnable(true);
  #endif

  return status;
}

void ledseqEnable(bool enable) {
  ledseqEnabled = enable;
}

bool ledseqRun(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = run;
  command.sequence = context;
  if (osMessageQueuePut(ledseqCmdQueue, &command, 0, 0) == osOK)
    return true;
  return false;
}

void ledseqRunBlocking(ledseqContext_t *context) {
  const led_t led = context->led;

  osMutexAcquire(ledseqMutex, osWaitForever);
  context->state = LEDSEQ_LIVE;  // Reset the seq. to its first step
  updateActive(led);
  osMutexRelease(ledseqMutex);

  // Run the first step if the new seq is the active sequence
  if (activeSeq[led] == context) {
    runLedseq(ledseqName[led]);
  }
}

void ledseqSetChargeLevel(const float chargeLevel) {
  int onTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA * chargeLevel;
  int offTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA - onTime;

  seq_charging.sequence[0].action = onTime;
  seq_charging.sequence[1].action = offTime;
}

bool ledseqStop(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = stop;
  command.sequence = context;
  if (osMessageQueuePut(ledseqCmdQueue, &command, 0, 0) == osOK) {
    return true;
  }
  return false;
}

void ledseqStopBlocking(ledseqContext_t *context) {
  const led_t led = context->led;

  osMutexAcquire(ledseqMutex, osWaitForever);
  context->state = LEDSEQ_STOP;  //Stop the seq.
  updateActive(led);
  osMutexRelease(ledseqMutex);

  //Run the next active sequence (if any...)
  runLedseq(ledseqName[led]);
}

/* Center of the led sequence machine. This function is executed by the FreeRTOS
 * timers and runs the sequences
 */
static void runLedseq(void *name) {
  if (!ledseqEnabled)
    return;

	int tim = ((char*) name)[4] - 0x30;
  ledseqContext_t* context = activeSeq[tim];

  if (NO_CONTEXT == context)
    return;

  bool leave = false;
  while (!leave) {
    if (context->state == LEDSEQ_STOP)
      return;
    const ledseqStep_t* step = &context->sequence[context->state];

    osMutexAcquire(ledseqMutex, osWaitForever);
    context->state++;
    led_t led = context->led;

    switch (step->action) {
      case LEDSEQ_LOOP:
        context->state = LEDSEQ_LIVE;
        break;
      case LEDSEQ_STOP:
        context->state = LEDSEQ_STOP;
        updateActive(led);
        osTimerStart(timer[tim], 1);
        break;
      default:  //The step is a LED action and a time
        ledSet(led, step->value);
        if (step->action == 0)
          break;
        else
				  osTimerStart(timer[tim], step->action);
        leave = true;
        break;
    }
    osMutexRelease(ledseqMutex);
  }
}

void ledseqRegisterSequence(ledseqContext_t* context) {
  context->state = LEDSEQ_STOP;
  context->nextContext = NO_CONTEXT;

	ledseqContext_t *last = sequences;

  if (sequences == NO_CONTEXT) {
    sequences = context;
  } else {
    while (last->nextContext != NO_CONTEXT) {
      last = last->nextContext;
      if (last == context)
        return;
    }
    last->nextContext = context;
  }
}

static void updateActive(led_t led) {
  activeSeq[led] = NO_CONTEXT;
  ledSet(led, false);

  for (ledseqContext_t *s = sequences; s != NO_CONTEXT; s = s->nextContext) {
    if (s->led == led && s->state != LEDSEQ_STOP) {
      activeSeq[led] = s;
      break;
    }
  }
}
