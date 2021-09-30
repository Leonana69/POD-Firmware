#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>
#include "config.h"
// Led polarity configuration constant
#define LED_POL_HIGH    1
#define LED_POL_LOW     0

// Hardware configuration
#define GREEN_L_Pol     LED_POL_LOW
#define GREEN_R_Pol     LED_POL_LOW
#define RED_L_Pol       LED_POL_LOW
#define RED_R_Pol       LED_POL_LOW
#define BLUE_L_Pol      LED_POL_HIGH

/* defined in config.h */
// #define GREEN_L_Pin       GPIO_PIN_0
// #define GREEN_L_GPIO_Port GPIOC
// #define RED_L_Pin         GPIO_PIN_1
// #define RED_L_GPIO_Port   GPIOC
// #define GREEN_R_Pin       GPIO_PIN_2
// #define GREEN_R_GPIO_Port GPIOC
// #define RED_R_Pin         GPIO_PIN_3
// #define RED_R_GPIO_Port   GPIOC
// #define BLUE_L_Pin        GPIO_PIN_2
// #define BLUE_L_GPIO_Port  GPIOD

#define LINK_LED         LED_GREEN_L
#define CHG_LED          LED_BLUE_L
#define LOWBAT_LED       LED_RED_R
#define LINK_DOWN_LED    LED_RED_L
#define SYS_LED          LED_RED_R
#define ERR_LED1         LED_RED_L
#define ERR_LED2         LED_RED_R

typedef enum { LED_BLUE_L = 0, LED_GREEN_L, LED_RED_L, LED_GREEN_R, LED_RED_R, LED_NUM } led_t;

void ledInit();
bool ledTest();

void ledBlink(int cnt);
// Clear all configured LEDs
void ledClearAll(void);

// Set all configured LEDs
void ledSetAll(void);

// Procedures to set the status of the LEDs
void ledSet(led_t led, bool value);
void ledToggle0();
#endif
