#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "eprintf.h"
#include "console.h"
#include "_usart.h"

#ifdef DEBUG_MODULE
#define DEBUG_FMT(fmt) DEBUG_MODULE ": " fmt
#endif

#ifndef DEBUG_FMT
#define DEBUG_FMT(FMT) FMT
#endif

#define DEBUG_PRINT_CONSOLE(FMT, ...) consolePrintf(DEBUG_FMT(FMT), ##__VA_ARGS__)
#define DEBUG_PRINT_UART(FMT, ...) eprintf(debugUartPutchar, FMT, ## __VA_ARGS__)

#ifdef DEBUG_PRINT_ON_UART
  #define DEBUG_PRINT(FMT, ...) eprintf(debugUartPutchar, FMT, ## __VA_ARGS__)
#else
  #define DEBUG_PRINT(FMT, ...) consolePrintf(DEBUG_FMT(FMT), ##__VA_ARGS__)
#endif

#endif