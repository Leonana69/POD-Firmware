#ifndef __UUSART_H__
#define __UUSART_H__

/*
 * USART
 */
#include "stdbool.h"
#include "usart.h"

int uartDebugPutchar(int c);
void _UART_Init(void);

#endif