#ifndef __CAL_H__
#define __CAL_H__

#include <stdint.h>
#include <unistd.h>

uint8_t calculate_cksum(void* data, size_t len);

#endif