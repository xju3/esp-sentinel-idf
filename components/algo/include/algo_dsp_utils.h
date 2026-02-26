/*
 * algo_dsp_utils.h - Internal DSP Utilities for Predictive Maintenance
 * 
 * Shared DSP operators used internally by the algorithm library.
 * These functions are NOT exposed to application layer.
 */

#ifndef ALGO_DSP_UTILS_H
#define ALGO_DSP_UTILS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

// Include the full definition of imu_raw_data_t
#include "algo_pdm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= *
 * 1. DATA INGESTION UTILITIES
 * ========================================================================= */

/**
 * @brief Extract single sample from raw IMU data
 * @param sample Raw IMU data sample
 * @param axis Axis to extract (0=X, 1=Y, 2=Z)
 * @return int16_t value in little-endian format
 */
static inline int16_t algo_extract_axis_raw(const imu_raw_data_t *sample, int axis)
{
    if (sample == NULL) {
        return 0;
    }
    
    int16_t raw_val;
    switch (axis) {
        case 0: raw_val = ((const int16_t *)sample)[1]; break; // x at offset 1
        case 1: raw_val = ((const int16_t *)sample)[2]; break; // y at offset 2  
        case 2: raw_val = ((const int16_t *)sample)[3]; break; // z at offset 3
        default: raw_val = 0; break;
    }
    
    // Convert big-endian to little-endian if needed
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap16(raw_val);
    #else
    return raw_val;
    #endif
}

/**
 * @brief Convert raw int16_t to float with sensitivity scaling
 * @param raw_val Raw sensor value (little-endian)
 * @param sensitivity Sensitivity factor (g/LSB or m/s²/LSB)
 * @return Scaled float value
 */
static inline float algo_scale_raw_to_float(int16_t raw_val, float sensitivity)
{
    return (float)raw_val * sensitivity;
}

/* ========================================================================= *
 * 2. FILTERING UTILITIES (using esp-dsp)
 * ========================================================================= */

/**
 * @brief Biquad filter structure for esp-dsp compatibility
 */
typedef struct {
    float coeffs[5];    // b0, b1, b2, a1, a2
    float delay[2];     // Delay line for filter state
} algo_biquad_t;

/**
 * @brief Initialize biquad filter coefficients for high-pass filter
 * @param bq Biquad structure to initialize
 * @param fs Sampling frequency (Hz)
 * @param fc Cutoff frequency (Hz)
 * @param Q Quality factor (default: 0.7071 for Butterworth)
 */
void algo_biquad_init_hp(algo_biquad_t *bq, float fs, float fc, float Q);

/**
 * @brief Initialize biquad filter coefficients for low-pass filter
 * @param bq Biquad structure to initialize
 * @param fs Sampling frequency (Hz)
 * @param fc Cutoff frequency (Hz)
 * @param Q Quality factor (default: 0.7071 for Butterworth)
 */
void algo_biquad_init_lp(algo_biquad_t *bq, float fs, float fc, float Q);

/**
 * @brief Process single sample through biquad filter
 * @param bq Biquad filter structure
 * @param input Input sample
 * @return Filtered output sample
 */
float algo_biquad_process(algo_biquad_t *bq, float input);

/**
 * @brief Process array through biquad filter (in-place)
 * @param bq Biquad filter structure
 * @param data Input/output array
 * @param count Number of samples
 */
void algo_biquad_process_array(algo_biquad_t *bq, float *data, size_t count);

/* ========================================================================= *
 * 3. TIME-FREQUENCY TRANSFORM UTILITIES
 * ========================================================================= */

/**
 * @brief Remove DC offset from signal (subtract mean)
 * @param data Input/output array
 * @param count Number of samples
 */
void algo_remove_dc_offset(float *data, size_t count);

/**
 * @brief Rectify signal (absolute value)
 * @param data Input/output array
 * @param count Number of samples
 */
void algo_rectify_signal(float *data, size_t count);

/**
 * @brief Integrate acceleration to velocity (trapezoidal rule)
 * @param accel Acceleration array (g or m/s²)
 * @param velocity Output velocity array (mm/s)
 * @param count Number of samples
 * @param dt Sampling interval (seconds)
 * @param scale_factor Scaling factor (e.g., 9.81 for g to m/s², 1000 for m/s to mm/s)
 */
void algo_integrate_accel_to_vel(const float *accel, float *velocity, 
                                 size_t count, float dt, float scale_factor);

/**
 * @brief Calculate power spectral density from magnitude spectrum
 * @param magnitude Magnitude spectrum (length N/2)
 * @param psd Output power spectral density (length N/2)
 * @param n_fft FFT size (N)
 * @param fs Sampling frequency (Hz)
 */
void algo_calc_psd(const float *magnitude, float *psd, size_t n_fft, float fs);

/* ========================================================================= *
 * 4. STATISTICAL UTILITIES
 * ========================================================================= */

/**
 * @brief Calculate mean of array
 * @param data Input array
 * @param count Number of samples
 * @return Mean value
 */
float algo_calc_mean(const float *data, size_t count);

/**
 * @brief Calculate variance of array
 * @param data Input array
 * @param count Number of samples
 * @param mean Pre-calculated mean
 * @return Variance
 */
float algo_calc_variance(const float *data, size_t count, float mean);

/**
 * @brief Calculate standard deviation of array
 * @param data Input array
 * @param count Number of samples
 * @param mean Pre-calculated mean
 * @return Standard deviation
 */
float algo_calc_std_dev(const float *data, size_t count, float mean);

#ifdef __cplusplus
}
#endif

#endif // ALGO_DSP_UTILS_H