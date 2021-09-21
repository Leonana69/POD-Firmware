#include "_gpio.h"

void HAL_RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *PORT) {
    if (PORT == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE ();
    else if (PORT == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE ();
    else if (PORT == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE ();
    else if (PORT == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE ();
    else if (PORT == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE ();
    else if (PORT == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE ();
}

void HAL_RCC_GPIO_CLK_DISABLE(GPIO_TypeDef *PORT) {
    if (PORT == GPIOA)
        __HAL_RCC_GPIOA_CLK_DISABLE ();
    else if (PORT == GPIOB)
        __HAL_RCC_GPIOB_CLK_DISABLE ();
    else if (PORT == GPIOC)
        __HAL_RCC_GPIOC_CLK_DISABLE ();
    else if (PORT == GPIOD)
        __HAL_RCC_GPIOD_CLK_DISABLE ();
    else if (PORT == GPIOE)
        __HAL_RCC_GPIOE_CLK_DISABLE ();
    else if (PORT == GPIOH)
        __HAL_RCC_GPIOH_CLK_DISABLE ();
}