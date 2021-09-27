#ifndef __UUSART_H__
#define __UUSART_H__

/*
 * USART
 */
#include "stdbool.h"
#include "usart.h"
#include "syslink.h"
int debugUartPutchar(int c);
void _UART_Init(void);

void nrfUartTxenIsr();
void nrfUartDmaIsr();

void nrfUartSendDataDmaBlocking(uint32_t size, uint8_t *data);
void nrfUartGetPacketBlocking(SyslinkPacket* packet);
void nrfUartIsr();
#endif