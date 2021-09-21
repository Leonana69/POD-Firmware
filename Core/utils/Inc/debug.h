#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "_usart.h"
#include "eprintf.h"

#ifndef DEBUG_FMT
#define DEBUG_FMT(FMT) FMT
#endif

#define DEBUG_PRINT(FMT, ...) eprintf(uartDebugPutchar, FMT, ## __VA_ARGS__)

#endif