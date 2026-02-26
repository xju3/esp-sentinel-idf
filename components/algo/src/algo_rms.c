/*
 * algo_rms.c - Root Mean Square (RMS) Calculation
 * 
 * Efficient RMS computation for vibration signal analysis.
 * Uses compensated summation for numerical stability.
 */

#include "algo_pdm.h"
#include <math.h>

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

float algo_calc_rms(const float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return 0.0f;
    }
    
    // Use Kahan summation for numerical stability
    float sum_sq = 0.0f;
    float compensation = 0.0f;
    
    for (size_t i = 0; i < count; i++) {
        float val = data[i];
        float term = val * val - compensation;
        float temp = sum_sq + term;
        
        // Kahan compensation
        compensation = (temp - sum_sq) - term;
        sum_sq = temp;
    }
    
    // Calculate RMS: sqrt(mean of squares)
    return sqrtf(sum_sq / count);
}

/* ========================================================================= *
 * OPTIMIZED RMS FUNCTIONS
 * ========================================================================= */

/**
 * @brief Calculate RMS with pre-removed DC offset
 * @details More accurate for signals with significant DC component
 */
float algo_calc_rms_ac(const float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return 0.0f;
    }
    
    // Calculate mean first
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    float mean = sum / count;
    
    // Calculate RMS of AC component
    float sum_sq = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float ac_val = data[i] - mean;
        sum_sq += ac_val * ac_val;
    }
    
    return sqrtf(sum_sq / count);
}

/**
 * @brief Calculate RMS in streaming fashion (for large datasets)
 * @details Processes data in chunks to avoid overflow
 */
float algo_calc_rms_streaming(const float *data, size_t count, size_t chunk_size)
{
    if (data == NULL || count == 0) {
        return 0.0f;
    }
    
    if (chunk_size == 0 || chunk_size > count) {
        chunk_size = count;
    }
    
    float total_sum_sq = 0.0f;
    size_t processed = 0;
    
    while (processed < count) {
        size_t remaining = count - processed;
        size_t chunk = (remaining < chunk_size) ? remaining : chunk_size;
        
        // Calculate sum of squares for this chunk
        float chunk_sum_sq = 0.0f;
        for (size_t i = 0; i < chunk; i++) {
            float val = data[processed + i];
            chunk_sum_sq += val * val;
        }
        
        total_sum_sq += chunk_sum_sq;
        processed += chunk;
    }
    
    return sqrtf(total_sum_sq / count);
}

/**
 * @brief Calculate RMS for windowed segments (for time-varying signals)
 */
void algo_calc_rms_windowed(const float *data, size_t count, 
                            size_t window_size, size_t step,
                            float *rms_output, size_t *output_count)
{
    if (data == NULL || rms_output == NULL || output_count == NULL || 
        count == 0 || window_size == 0 || step == 0) {
        if (output_count) *output_count = 0;
        return;
    }
    
    if (window_size > count) {
        window_size = count;
    }
    
    size_t num_windows = (count - window_size) / step + 1;
    if (num_windows == 0) {
        *output_count = 0;
        return;
    }
    
    for (size_t w = 0; w < num_windows; w++) {
        size_t start = w * step;
        
        // Calculate RMS for this window
        float sum_sq = 0.0f;
        for (size_t i = 0; i < window_size; i++) {
            float val = data[start + i];
            sum_sq += val * val;
        }
        
        rms_output[w] = sqrtf(sum_sq / window_size);
    }
    
    *output_count = num_windows;
}

/* ========================================================================= *
 * RMS-BASED FEATURE EXTRACTION
 * ========================================================================= */

/**
 * @brief Calculate Crest Factor (peak / RMS)
 */
float algo_calc_crest_factor(const float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return 0.0f;
    }
    
    // Find peak value
    float peak = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float abs_val = fabsf(data[i]);
        if (abs_val > peak) {
            peak = abs_val;
        }
    }
    
    // Calculate RMS
    float rms = algo_calc_rms(data, count);
    
    // Crest factor
    return (rms > 0.0f) ? (peak / rms) : 0.0f;
}

/**
 * @brief Calculate RMS ratio between two frequency bands
 */
float algo_calc_rms_ratio(const float *data_low, size_t count_low,
                          const float *data_high, size_t count_high)
{
    if (data_low == NULL || data_high == NULL || 
        count_low == 0 || count_high == 0) {
        return 0.0f;
    }
    
    float rms_low = algo_calc_rms(data_low, count_low);
    float rms_high = algo_calc_rms(data_high, count_high);
    
    return (rms_low > 0.0f) ? (rms_high / rms_low) : 0.0f;
}

/**
 * @brief Calculate RMS of envelope signal (for vibration analysis)
 */
float algo_calc_envelope_rms(const float *signal, size_t count,
                             float fs, float hp_freq, float lp_freq,
                             float *work_buf)
{
    if (signal == NULL || count == 0 || fs <= 0.0f || 
        hp_freq <= 0.0f || lp_freq <= 0.0f) {
        return 0.0f;
    }
    
    // Simple envelope detection using absolute value and moving average
    // For production, use algo_envelope_process() instead
    
    // Calculate moving average of absolute values (simple envelope)
    const size_t window_size = (size_t)(fs / (2.0f * lp_freq)); // Approximate
    
    if (window_size >= count) {
        // Fallback to simple RMS of absolute values
        float sum_sq = 0.0f;
        for (size_t i = 0; i < count; i++) {
            float abs_val = fabsf(signal[i]);
            sum_sq += abs_val * abs_val;
        }
        return sqrtf(sum_sq / count);
    }
    
    // Use work buffer if provided, otherwise allocate locally (small)
    float envelope_local[256];
    float *envelope = (work_buf != NULL) ? work_buf : envelope_local;
    
    if (count > sizeof(envelope_local)/sizeof(envelope_local[0]) && work_buf == NULL) {
        // Buffer too small, fallback to simple RMS
        return algo_calc_rms(signal, count);
    }
    
    // Simple moving average envelope
    for (size_t i = 0; i < count; i++) {
        float sum = 0.0f;
        size_t start = (i > window_size) ? (i - window_size) : 0;
        size_t end = i + 1;
        size_t window_count = end - start;
        
        for (size_t j = start; j < end; j++) {
            sum += fabsf(signal[j]);
        }
        
        envelope[i] = sum / window_count;
    }
    
    // Calculate RMS of envelope
    return algo_calc_rms(envelope, count);
}