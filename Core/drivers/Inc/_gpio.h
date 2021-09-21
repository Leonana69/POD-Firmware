#ifndef __UGPIO_H__
#define __UGPIO_H__

/*
 * GPIO
 */
#include "gpio.h"

void HAL_RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *PORT);
void HAL_RCC_GPIO_CLK_DISABLE(GPIO_TypeDef *PORT);

#endif