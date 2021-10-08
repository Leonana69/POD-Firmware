#ifndef __CAL_H__
#define __CAL_H__

#include <stdint.h>
#include <unistd.h>
#include <math.h>

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

uint8_t calculate_cksum(void* data, size_t len);

uint16_t single2half(float number);
float half2single(uint16_t number);

uint16_t limitUint16(int32_t value);
float constrain(float value, const float minVal, const float maxVal);
float deadband(float value, const float threshold);

/**
 * @brief CRC32 context
 * 
 * Should be considered as an opaque object containing the current
 * state of the CRC32 checksum calculation.
 * 
 */
typedef struct {
    uint32_t remainder;
} crc32Context_t;

/**
 * @brief Initialize CRC32 context
 * 
 * This function must be called before using the context.
 * It can be called on an already initialized context to reset it
 * and calculate a new CRC32.
 * 
 * @param context Context to initialize
 */
void crc32ContextInit(crc32Context_t *context);

/**
 * @brief Update checksum with new data
 * 
 * Can be called multiple times to add data to checksum.
 * 
 * @param context an initialized context
 * @param data Data buffer to add to the checksum
 * @param size Size in byte of the data buffer
 */
void crc32Update(crc32Context_t *context, const void* data, size_t size);

/**
 * @brief Generate and returns the CRC32 checksum
 * 
 * @param context An initialized context
 * @return the CRC32 checksum
 */
uint32_t crc32Out(const crc32Context_t *context);

/**
 * @brief Utility function to calculate the checksum of a buffer in one call
 * 
 * This function calls contextInit, update and out in sequence to generate the
 * checksum of a buffer. It is a conveniance function to calculate the checksum
 * of a buffer that is already complete in memory
 * 
 * @param buffer Data buffer that needs to be checksumed
 * @param size Size in byte of the data buffer
 * @return The CRC32 checksum
 */
uint32_t crc32CalculateBuffer(const void* buffer, size_t size);

uint32_t quaternionCompress(float const q[4]);
void quaternionDecompress(uint32_t cq, float q[4]);
float fConstrain(float val, float vMin, float vMax);
float capAngle(float angle);
int16_t capValueInt16(float in);

static inline float radians(float degrees) { return (M_PI / 180.0f) * degrees; }
static inline float degrees(float radians) { return (180.0f / M_PI) * radians; }
#endif