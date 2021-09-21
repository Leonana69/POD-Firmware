#include "_gpio.h"
#include "_usart.h"
#include "config.h"

void _UART_Init(void) {
  // enable flow control GPIO
  HAL_RCC_GPIO_CLK_ENABLE(NRF_FC_PIN_PORT);
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = NRF_FC_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(NRF_FC_PIN_PORT, &GPIO_InitStruct);
}

int uartDebugPutchar(int c) {
    HAL_UART_Transmit(&uartDebug, (uint8_t*) &c, 1, 100);
    return (unsigned char) c;
}