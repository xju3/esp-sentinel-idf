/* 
 * algo_pdm.h - Predictive Maintenance Algorithm Library
 * 
 * Industrial-grade DSP library for vibration analysis on ESP32-S3/ESP32.
 * Pure mathematical functions with zero external dependencies (except esp-dsp).
 * 
 * Key Principles:
 * 1. Zero business coupling - No #include of project internal headers
 * 2. Zero dynamic memory allocation - All buffers allocated by caller
 * 3. Hardware acceleration priority - Use esp-dsp for FFT, FIR/IIR
 * 4. Efficient data ingestion - Handle packed big-endian int16_t DMA data
 */

#ifndef ALGO_PDM_H
#define ALGO_PDM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= *
 * 1. PHYSICAL LAYER DATA DEFINITIONS
 * ========================================================================= */

// imu_raw_data_t is defined in algo_types.h
#include "algo_types.h"

/**
 * @brief Axis selection for data processing
 */
typedef enum {
    ALGO_AXIS_X = 0,
    ALGO_AXIS_Y = 1,
    ALGO_AXIS_Z = 2
} algo_axis_t;

/* ========================================================================= *
 * 2. ALGORITHM CONTEXTS - Allocated by caller
 * ========================================================================= */

/**
 * @brief Welford online statistics context
 * O(1) space complexity, no large arrays needed
 */
typedef struct {
    uint32_t count;     // Number of samples processed
    double mean;        // Current mean value
    double m2;          // Sum of squared differences from mean
    float min_val;      // Minimum value observed
    float max_val;      // Maximum value observed
} algo_welford_ctx_t;

/**
 * @brief Envelope analysis configuration and state
 * Uses esp-dsp biquad filters for high-pass and low-pass filtering
 */
typedef struct {
    float fs_hz;            // Sampling frequency (Hz)
    float hp_cutoff_hz;     // High-pass cutoff frequency (Hz)
    float lp_cutoff_hz;     // Low-pass cutoff frequency (Hz)
    
    // Internal biquad filter states (for esp-dsp compatibility)
    float hp_delay[2];      // High-pass filter delay line
    float hp_coeffs[5];     // b0, b1, b2, a1, a2
    
    float lp_delay[2];      // Low-pass filter delay line  
    float lp_coeffs[5];     // b0, b1, b2, a1, a2
} algo_envelope_ctx_t;

/**
 * @brief FFT context for hardware-accelerated frequency analysis
 */
typedef struct {
    size_t max_fft_size;    // Maximum FFT size supported
    bool initialized;       // Whether FFT engine is initialized
} algo_fft_ctx_t;

/* ========================================================================= *
 * 3. TOP-LEVEL API FUNCTIONS (Application Interface Layer)
 * ========================================================================= */

/**
 * @brief [DATA INGESTION] Efficient raw data ingestion operator
 * @details Extracts specified axis -> big-endian to little-endian conversion -> 
 *          scaling by sensitivity -> stores in caller-provided float array
 * @param src Pointer to raw IMU data array
 * @param count Number of samples to process
 * @param axis Axis to extract (X, Y, or Z)
 * @param sensitivity Sensitivity factor (g/LSB or m/s²/LSB)
 * @param out_buf Caller-allocated output buffer (size >= count)
 */
void algo_ingest_axis(const imu_raw_data_t *src, 
                      size_t count, 
                      algo_axis_t axis, 
                      float sensitivity, 
                      float *out_buf);

/**
 * @brief [WELFORD] Streaming statistical update
 * @details Directly processes raw data without intermediate float arrays.
 *          Extremely memory efficient - O(1) space complexity.
 * @param ctx Welford context (must be initialized with zeros)
 * @param src Raw IMU data array
 * @param count Number of samples to process
 * @param axis Axis to process
 * @param sensitivity Sensitivity factor
 */
void algo_welford_update(algo_welford_ctx_t *ctx, 
                         const imu_raw_data_t *src, 
                         size_t count, 
                         algo_axis_t axis, 
                         float sensitivity);

/**
 * @brief [WELFORD] Get current statistics
 * @param ctx Welford context
 * @param mean Output: current mean value
 * @param variance Output: current variance
 * @param std_dev Output: current standard deviation
 */
void algo_welford_get_stats(const algo_welford_ctx_t *ctx,
                            float *mean, float *variance, float *std_dev);

/**
 * @brief [RMS] Calculate Root Mean Square of float array
 * @param data Input data array
 * @param count Number of samples
 * @return RMS value
 */
float algo_calc_rms(const float *data, size_t count);

/**
 * @brief [KURTOSIS] Calculate kurtosis of float array
 * @param data Input data array
 * @param count Number of samples
 * @param mean Pre-calculated mean (can be from Welford)
 * @param std_dev Pre-calculated standard deviation (can be from Welford)
 * @return Kurtosis value (Normal distribution = 3.0)
 */
float algo_calc_kurtosis(const float *data, size_t count, float mean, float std_dev);

/**
 * @brief [ENVELOPE] Initialize envelope analyzer
 * @param ctx Envelope context to initialize
 * @param fs Sampling frequency (Hz)
 * @param hp_freq High-pass cutoff frequency (Hz)
 * @param lp_freq Low-pass cutoff frequency (Hz)
 */
void algo_envelope_init(algo_envelope_ctx_t *ctx, float fs, float hp_freq, float lp_freq);

/**
 * @brief [ENVELOPE] Complete envelope analysis pipeline
 * @details Ingest -> High-pass filter -> Rectify -> Low-pass filter
 * @param ctx Initialized envelope context
 * @param raw_src Raw IMU data array
 * @param count Number of samples to process
 * @param axis Axis to process
 * @param sensitivity Sensitivity factor
 * @param work_buf Caller-allocated work buffer (size >= count)
 * @param out_buf Caller-allocated output buffer (size >= count)
 */
void algo_envelope_process(algo_envelope_ctx_t *ctx,
                           const imu_raw_data_t *raw_src,
                           size_t count,
                           algo_axis_t axis,
                           float sensitivity,
                           float *work_buf,
                           float *out_buf);

/**
 * @brief [FFT] Initialize FFT engine (call once at startup)
 * @param max_fft_size Maximum FFT size to support (must be power of 2)
 */
void algo_fft_init(size_t max_fft_size);

/**
 * @brief [FFT] Execute real FFT and compute magnitude spectrum
 * @param input Real input signal (length N)
 * @param n FFT size (must be power of 2 and <= max_fft_size)
 * @param work_buf Work buffer (length 2*N float)
 * @param output Magnitude spectrum output (length N/2 float)
 */
void algo_fft_execute(const float *input, size_t n, float *work_buf, float *output);

#ifdef __cplusplus
}
#endif

#endif // ALGO_PDM_H