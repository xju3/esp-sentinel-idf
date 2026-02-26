/**
 * @file algo_types.h
 * @brief Algorithm layer common data types
 * 
 * This file defines common data types used across the algorithm layer,
 * including IMU raw data structures.
 */

#ifndef ALGO_TYPES_H
#define ALGO_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Three-axis accelerometer raw data (Raw Data)
 * @note Must remain as int16_t to minimize DMA transfer bandwidth and PSRAM usage
 * __attribute__((packed)) ensures the structure size is exactly 8 bytes.
 * This is crucial for DMA directly mapping SPI hardware streams to memory structures.
 */
typedef struct
{
    uint8_t header; // Packet header (identifies data validity, etc.)
    int16_t x;      // Raw X-axis (note: internal big-endian format)
    int16_t y;      // Raw Y-axis
    int16_t z;      // Raw Z-axis
    int8_t temp;    // 8-bit truncated temperature auxiliary data
} __attribute__((packed)) imu_raw_data_t;

#ifdef __cplusplus
}
#endif

#endif // ALGO_TYPES_H
