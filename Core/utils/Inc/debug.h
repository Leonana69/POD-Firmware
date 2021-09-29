#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "eprintf.h"
#include "console.h"

#ifndef DEBUG_FMT
#define DEBUG_FMT(FMT) FMT
#endif

#ifdef DEBUG_PRINT_ON_UART
  #include "_usart.h"
  #define DEBUG_PRINT(FMT, ...) eprintf(debugUartPutchar, FMT, ## __VA_ARGS__)
#else
	#include "_usart.h"
  // #define DEBUG_PRINT(FMT, ...) eprintf(debugUartPutchar, FMT, ## __VA_ARGS__)
  #define DEBUG_PRINT(FMT, ...)
  #define DEBUG_PRINT_UART(FMT, ...) eprintf(debugUartPutchar, FMT, ## __VA_ARGS__)
	#define DEBUG_PRINT_CONSOLE(FMT, ...) consolePrintf(DEBUG_FMT(FMT), ##__VA_ARGS__)
#endif

#endif