#ifndef ALGO_KURTOSIS_H
#define ALGO_KURTOSIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} vib_kurtosis_t;

/**
 * @brief Calculate Kurtosis for 3-axis vibration data using ESP-DSP
 * 
 * @param x Pointer to X-axis data
 * @param y Pointer to Y-axis data
 * @param z Pointer to Z-axis data
 * @param length Number of samples per axis
 * @return vib_kurtosis_t Result containing kurtosis for each axis
 */
vib_kurtosis_t algo_kurtosis_calculate(const float *x, const float *y, const float *z, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // ALGO_KURTOSIS_H