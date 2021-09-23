#include "led.h"
#include "_gpio.h"

#include "cmsis_os.h"


static GPIO_TypeDef* led_port[] = {
  [LED_BLUE_L]  = BLUE_L_GPIO_Port,
  [LED_GREEN_L] = GREEN_L_GPIO_Port,
  [LED_RED_L] 	= RED_L_GPIO_Port,
  [LED_GREEN_R] = GREEN_R_GPIO_Port,
  [LED_RED_R] 	= RED_R_GPIO_Port,
};

static unsigned int led_pin[] = {
  [LED_BLUE_L]  = BLUE_L_Pin,
  [LED_GREEN_L] = GREEN_L_Pin,
  [LED_RED_L]   = RED_L_Pin,
  [LED_GREEN_R] = GREEN_R_Pin,
  [LED_RED_R]   = RED_R_Pin,
};

static int led_polarity[] = {
  [LED_BLUE_L]  = BLUE_L_Pol,
  [LED_GREEN_L] = GREEN_L_Pol,
  [LED_RED_L]   = RED_L_Pol,
  [LED_GREEN_R] = GREEN_R_Pol,
  [LED_RED_R]   = RED_R_Pol,
};

static bool isInit = false;

//Initialize the green led pin as output
void ledInit() {
  if (isInit)
    return;
	
	for (int i = 0; i < LED_NUM; i++)
		ledSet(i, 0);

  isInit = true;
}

bool ledTest(void) {
  ledSet(LED_GREEN_L, 1);
  ledSet(LED_GREEN_R, 1);
  ledSet(LED_RED_L, 0);
  ledSet(LED_RED_R, 0);
  osDelay(250);
  ledSet(LED_GREEN_L, 0);
  ledSet(LED_GREEN_R, 0);
  ledSet(LED_RED_L, 1);
  ledSet(LED_RED_R, 1);
  osDelay(250);

  // LED test end
  ledClearAll();
  ledSet(LED_BLUE_L, 1);

  return isInit;
}

void ledClearAll(void) {
  for (int i = 0; i < LED_NUM; i++)
    ledSet(i, 0);
}

void ledSetAll(void) {
  for (int i = 0; i < LED_NUM; i++)
    ledSet(i, 1);
}

void ledSet(led_t led, bool value) {
  if (led > LED_NUM)
    return;
	HAL_GPIO_WritePin(led_port[led], led_pin[led], value == led_polarity[led]);
}
