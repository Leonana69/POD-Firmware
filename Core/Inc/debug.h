#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "usart.h"
#include "main.h"
#include "eprintf.h"

#ifndef DEBUG_FMT
#define DEBUG_FMT(FMT) FMT
#endif

#ifdef DEBUG_UART3
#define DEBUG_PRINT(FMT, ...) eprintf(uart3Putchar, FMT, ## __VA_ARGS__)
#endif

#endif