#include "cal.h"
#include <math.h>

// TODO: add descriptions
uint8_t calculate_cksum(void* data, size_t len) {
  unsigned char* c = data;
  unsigned char cksum = 0;
  
  for (int i = 0; i < len; i++)
    cksum += *(c++);

  return cksum;
}

uint16_t single2half(float number) {
    uint32_t num = *((uint32_t *)&number);
    uint32_t s = num >> 31;
    uint32_t e = (num >> 23) & 0x0FF;

    if ((e == 255) && (num & 0x007fffff))
        return 0x7E00; // NaN
    if (e > (127 + 15))
        return s ? 0xFC00 : 0x7C00;  //+/- inf
    if (e < (127 - 15))
        return 0; //Do not handle generating subnormalised representation

    return (s << 15) | ((e - 127 + 15) << 10) | (((num >> 13) & 0x3FF) + ((num >> 12) & 0x01));
}

float half2single(uint16_t number) {
    uint32_t fp32;
    uint32_t s = number >> 15;
    uint32_t e = (number >> 10) & 0x01F;

    //All binary16 can be mapped in a binary32
    if (e == 0)
        e = 15 - 127;
    if (e == 0x1F) {
        if (number & 0x03FF)
            fp32 = 0x7FC00000; // NaN
        else
            fp32 = s ? 0xFF800000 : 0x7F800000;  //+/- inf
    } else
        fp32 = (s << 31) | ((e + 127 - 15) << 23) | ((number & 0x3ff) << 13);

    return *(float *)&fp32;
}

float constrain(float value, const float minVal, const float maxVal) {
  return fminf(maxVal, fmaxf(minVal, value));
}

float deadband(float value, const float threshold) {
  if (fabsf(value) < threshold)
    value = 0;
  else if (value > 0)
    value -= threshold;
  else if (value < 0)
    value += threshold;
  return value;
}

/**
 * Core calculation code is:
 * 
 * Copyright (C) 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 */


#include "static_mem.h"
#define POLYNOMIAL              0xEDB88320
#define CHECK_VALUE             0xCBF43926
#define INITIAL_REMAINDER       0xFFFFFFFF
#define FINAL_XOR_VALUE         0xFFFFFFFF
#define RESIDUE                 0xDEBB20e3

// Internal functions
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable);
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable);
static void crcTableInit(uint32_t* crcTable);

NO_DMA_CCM_SAFE_ZERO_INIT static uint32_t crcTable[256];
static bool crcTableInitialized = false;

// *** Public API ***

void crc32ContextInit(crc32Context_t *context) {
  // Lazy static ...
  if (crcTableInitialized == false) {
    // initialize crcTable
    crcTableInit(crcTable);
    crcTableInitialized = true;
  }

  context->remainder = INITIAL_REMAINDER;
}

void crc32Update(crc32Context_t *context, const void* data, size_t size) {
  context->remainder = crcByByte(data, size, context->remainder, crcTable);
}

uint32_t crc32Out(const crc32Context_t *context) {
  return context->remainder ^ FINAL_XOR_VALUE;
}

uint32_t crc32CalculateBuffer(const void* buffer, size_t size) {
  crc32Context_t ctx;

  crc32ContextInit(&ctx);
  crc32Update(&ctx, buffer, size);
  return crc32Out(&ctx);
}

// *** Core calculation from Bosh ***

/* bit-wise crc calculation */
static uint32_t crcByBit(const uint8_t* message, uint32_t bytesToProcess,
             uint32_t remainder) {
  for (unsigned int byte = 0; byte < bytesToProcess; ++byte) {
    remainder ^= *(message + byte);

    for (uint8_t bit = 8; bit > 0; --bit) {
      /* reflect is realized by mirroring algorithm
        * LSB is first to be processed */
      if (remainder & 1)
        remainder = (remainder >> 1) ^ POLYNOMIAL;
      else
        remainder = (remainder >> 1);
    }
  }
  return remainder;
}

/* byte-wise crc calculation, requires an initialized crcTable
 * this is factor 8 faster and should be used if multiple crcs
 * have to be calculated */
static uint32_t crcByByte(const uint8_t* message, uint32_t bytesToProcess,
              uint32_t remainder, uint32_t* crcTable) {
  uint8_t data;
  for (int byte = 0; byte < bytesToProcess; ++byte) {
    data = (*(message + byte) ^ remainder);
    remainder = *(crcTable + data) ^ (remainder >> 8);
  }
  return remainder;
}

/* creates a lookup-table which is necessary for the crcByByte function */
static void crcTableInit(uint32_t* crcTable) {
  uint8_t dividend = ~0;
  /* fill the table by bit-wise calculations of checksums
   * for each possible dividend */
  do {
      *(crcTable + dividend) = crcByBit(&dividend, 1, 0);
  } while(dividend-- > 0);
}

uint32_t quaternionCompress(float const q[4]) {
  uint8_t l = 0;
  uint8_t s;
  for (uint8_t i = 0; i < 4; i++) {
    if (fabsf(q[i]) > fabsf(q[l]))
      l = i;
  }
  uint32_t cq = l & 0x3;
  s = q[l] < 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (i != l) {
      uint8_t neg = (q[i] < 0) ^ s;
      uint16_t val = ((1 << 9) - 1) * fabsf(q[i]) * M_SQRT2 + 0.5f;
      cq = (cq << 10) | (neg << 9) | (val & 0x1FF);
    }
  }
  
  return cq;
}

void quaternionDecompress(uint32_t cq, float q[4]) {
  const static uint16_t mask = (1 << 9) - 1;
  const static float dmask = 1.0 / (float) mask;
  uint8_t l = cq >> 30;
  float squareSum = 1.0;
  for (int8_t i = 3; i >= 0; i--) {
    if (i != l) {
      uint8_t neg = (cq >> 9) & 0x1;
      uint16_t val = cq & mask;
      q[i] = (M_SQRT1_2 * (float) val) * dmask;
      if (neg) q[i] = -q[i];
      cq = cq >> 10;
      squareSum -= q[i] * q[i];
    }
  }
  q[l] = sqrtf(squareSum);
}

float fConstrain(float val, const float vMin, const float vMax) {
  return fminf(vMax, fmaxf(vMin, val));
}

float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f)
    result -= 360.0f;

  while (result < -180.0f)
    result += 360.0f;

  return result;
}

int16_t capValueInt16(float value) {
  if (value > INT16_MAX)
    return INT16_MAX;
  else if (value < INT16_MIN)
    return INT16_MIN;
  else return (int16_t)value;
}

uint16_t capValueUint16(int32_t value) {
  if (value > UINT16_MAX)
    value = UINT16_MAX;
  else if (value < 0)
    value = 0;
  return (uint16_t)value;
}