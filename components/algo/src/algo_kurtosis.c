/*
 * algo_kurtosis.c - Kurtosis Calculation for Vibration Analysis
 * 
 * Kurtosis measures the "tailedness" of probability distribution.
 * High kurtosis indicates impulsive signals (common in bearing faults).
 * Normal distribution has kurtosis = 3.0.
 */

#include "algo_pdm.h"
#include <math.h>

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

float algo_calc_kurtosis(const float *data, size_t count, float mean, float std_dev)
{
    // Return normal distribution kurtosis for invalid input
    if (data == NULL || count < 4 || std_dev <= 0.0f) {
        return 3.0f; 
    }
    
    // Calculate fourth central moment
    float sum_fourth = 0.0f;
    float variance = std_dev * std_dev;
    
    for (size_t i = 0; i < count; i++) {
        float diff = data[i] - mean;
        float diff_sq = diff * diff;
        sum_fourth += diff_sq * diff_sq; // diff^4
    }
    
    // Kurtosis = (fourth moment) / (variance^2)
    float fourth_moment = sum_fourth / count;
    float kurtosis = fourth_moment / (variance * variance);
    
    return kurtosis;
}

/* ========================================================================= *
 * OPTIMIZED KURTOSIS FUNCTIONS
 * ========================================================================= */

/**
 * @brief Calculate kurtosis with pre-computed statistics
 * @details More efficient when mean and std_dev are already available
 */
float algo_calc_kurtosis_precomputed(const float *data, size_t count, 
                                     float mean, float variance)
{
    if (data == NULL || count < 4 || variance <= 0.0f) {
        return 3.0f;
    }
    
    float sum_fourth = 0.0f;
    
    for (size_t i = 0; i < count; i++) {
        float diff = data[i] - mean;
        float diff_sq = diff * diff;
        sum_fourth += diff_sq * diff_sq;
    }
    
    float fourth_moment = sum_fourth / count;
    return fourth_moment / (variance * variance);
}

/**
 * @brief Calculate excess kurtosis (kurtosis - 3)
 * @details Excess kurtosis = 0 for normal distribution
 */
float algo_calc_excess_kurtosis(const float *data, size_t count, 
                                float mean, float std_dev)
{
    float kurtosis = algo_calc_kurtosis(data, count, mean, std_dev);
    return kurtosis - 3.0f;
}

/**
 * @brief Calculate kurtosis from raw data (computes mean and std_dev internally)
 */
float algo_calc_kurtosis_raw(const float *data, size_t count)
{
    if (data == NULL || count < 4) {
        return 3.0f;
    }
    
    // Calculate mean
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    float mean = sum / count;
    
    // Calculate variance
    float sum_sq_diff = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float diff = data[i] - mean;
        sum_sq_diff += diff * diff;
    }
    float variance = sum_sq_diff / (count - 1); // Sample variance
    
    if (variance <= 0.0f) {
        return 3.0f;
    }
    
    // Calculate kurtosis
    return algo_calc_kurtosis_precomputed(data, count, mean, variance);
}

/* ========================================================================= *
 * KURTOSIS-BASED FAULT DETECTION
 * ========================================================================= */

/**
 * @brief Detect impulsive faults using kurtosis threshold
 * @return true if kurtosis exceeds threshold (indicating fault)
 */
bool algo_detect_impulsive_fault(const float *data, size_t count,
                                 float mean, float std_dev,
                                 float threshold)
{
    if (data == NULL || count < 4 || std_dev <= 0.0f) {
        return false;
    }
    
    float kurtosis = algo_calc_kurtosis(data, count, mean, std_dev);
    float excess_kurtosis = kurtosis - 3.0f;
    
    return (excess_kurtosis > threshold);
}

/**
 * @brief Calculate kurtosis for windowed segments
 */
void algo_calc_kurtosis_windowed(const float *data, size_t count,
                                 size_t window_size, size_t step,
                                 float *kurtosis_output, size_t *output_count)
{
    if (data == NULL || kurtosis_output == NULL || output_count == NULL ||
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
        
        // Calculate mean for this window
        float window_sum = 0.0f;
        for (size_t i = 0; i < window_size; i++) {
            window_sum += data[start + i];
        }
        float window_mean = window_sum / window_size;
        
        // Calculate variance for this window
        float window_sum_sq_diff = 0.0f;
        for (size_t i = 0; i < window_size; i++) {
            float diff = data[start + i] - window_mean;
            window_sum_sq_diff += diff * diff;
        }
        float window_variance = window_sum_sq_diff / (window_size - 1);
        
        if (window_variance <= 0.0f) {
            kurtosis_output[w] = 3.0f;
            continue;
        }
        
        // Calculate kurtosis for this window
        float window_sum_fourth = 0.0f;
        for (size_t i = 0; i < window_size; i++) {
            float diff = data[start + i] - window_mean;
            float diff_sq = diff * diff;
            window_sum_fourth += diff_sq * diff_sq;
        }
        
        float fourth_moment = window_sum_fourth / window_size;
        kurtosis_output[w] = fourth_moment / (window_variance * window_variance);
    }
    
    *output_count = num_windows;
}

/**
 * @brief Calculate kurtosis of envelope signal (for bearing fault detection)
 */
float algo_calc_envelope_kurtosis(const float *signal, size_t count,
                                  float *work_buf)
{
    if (signal == NULL || count < 4) {
        return 3.0f;
    }
    
    // Simple envelope using absolute value
    // For production, use proper envelope detection
    
    // Use work buffer if provided
    float envelope_local[256];
    float *envelope = (work_buf != NULL) ? work_buf : envelope_local;
    
    if (count > sizeof(envelope_local)/sizeof(envelope_local[0]) && work_buf == NULL) {
        // Buffer too small, use absolute values directly
        return algo_calc_kurtosis_raw(signal, count);
    }
    
    // Calculate absolute values (simple envelope)
    for (size_t i = 0; i < count; i++) {
        envelope[i] = fabsf(signal[i]);
    }
    
    // Calculate kurtosis of envelope
    return algo_calc_kurtosis_raw(envelope, count);
}

/**
 * @brief Calculate combined RMS and kurtosis feature
 * @details Useful for machine learning feature extraction
 */
void algo_calc_rms_kurtosis_features(const float *data, size_t count,
                                     float *rms, float *kurtosis)
{
    if (data == NULL || count == 0) {
        if (rms) *rms = 0.0f;
        if (kurtosis) *kurtosis = 3.0f;
        return;
    }
    
    // Calculate mean
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    float mean = sum / count;
    
    // Calculate variance and RMS simultaneously
    float sum_sq = 0.0f;
    float sum_sq_diff = 0.0f;
    float sum_fourth = 0.0f;
    
    for (size_t i = 0; i < count; i++) {
        float val = data[i];
        float diff = val - mean;
        float diff_sq = diff * diff;
        
        sum_sq += val * val;
        sum_sq_diff += diff_sq;
        sum_fourth += diff_sq * diff_sq;
    }
    
    // Calculate RMS
    if (rms) {
        *rms = sqrtf(sum_sq / count);
    }
    
    // Calculate kurtosis
    if (kurtosis) {
        if (count >= 4 && sum_sq_diff > 0.0f) {
            float variance = sum_sq_diff / (count - 1);
            float fourth_moment = sum_fourth / count;
            *kurtosis = fourth_moment / (variance * variance);
        } else {
            *kurtosis = 3.0f;
        }
    }
}