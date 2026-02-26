/*
 * algo_dsp_utils.c - Internal DSP Utilities Implementation
 * 
 * Shared DSP operators used internally by the algorithm library.
 */

#include "algo_dsp_utils.h"
#include <math.h>
#include <string.h>

/* ========================================================================= *
 * FILTERING UTILITIES IMPLEMENTATION
 * ========================================================================= */

void algo_biquad_init_hp(algo_biquad_t *bq, float fs, float fc, float Q)
{
    if (bq == NULL || fs <= 0.0f || fc <= 0.0f || Q <= 0.0f) {
        return;
    }
    
    // Normalize frequency
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);
    
    // High-pass filter coefficients
    float b0 = (1.0f + cos_w0) / 2.0f;
    float b1 = -(1.0f + cos_w0);
    float b2 = (1.0f + cos_w0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    // Normalize by a0
    bq->coeffs[0] = b0 / a0;
    bq->coeffs[1] = b1 / a0;
    bq->coeffs[2] = b2 / a0;
    bq->coeffs[3] = a1 / a0;
    bq->coeffs[4] = a2 / a0;
    
    // Reset delay line
    bq->delay[0] = 0.0f;
    bq->delay[1] = 0.0f;
}

void algo_biquad_init_lp(algo_biquad_t *bq, float fs, float fc, float Q)
{
    if (bq == NULL || fs <= 0.0f || fc <= 0.0f || Q <= 0.0f) {
        return;
    }
    
    // Normalize frequency
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    float cos_w0 = cosf(w0);
    
    // Low-pass filter coefficients
    float b0 = (1.0f - cos_w0) / 2.0f;
    float b1 = 1.0f - cos_w0;
    float b2 = (1.0f - cos_w0) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    // Normalize by a0
    bq->coeffs[0] = b0 / a0;
    bq->coeffs[1] = b1 / a0;
    bq->coeffs[2] = b2 / a0;
    bq->coeffs[3] = a1 / a0;
    bq->coeffs[4] = a2 / a0;
    
    // Reset delay line
    bq->delay[0] = 0.0f;
    bq->delay[1] = 0.0f;
}

float algo_biquad_process(algo_biquad_t *bq, float input)
{
    if (bq == NULL) {
        return 0.0f;
    }
    
    // Direct form II biquad implementation
    float w = input - bq->coeffs[3] * bq->delay[0] - bq->coeffs[4] * bq->delay[1];
    float output = bq->coeffs[0] * w + bq->coeffs[1] * bq->delay[0] + bq->coeffs[2] * bq->delay[1];
    
    // Update delay line
    bq->delay[1] = bq->delay[0];
    bq->delay[0] = w;
    
    return output;
}

void algo_biquad_process_array(algo_biquad_t *bq, float *data, size_t count)
{
    if (bq == NULL || data == NULL || count == 0) {
        return;
    }
    
    for (size_t i = 0; i < count; i++) {
        data[i] = algo_biquad_process(bq, data[i]);
    }
}

/* ========================================================================= *
 * TIME-FREQUENCY TRANSFORM UTILITIES IMPLEMENTATION
 * ========================================================================= */

void algo_remove_dc_offset(float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return;
    }
    
    // Calculate mean
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    float mean = sum / count;
    
    // Subtract mean
    for (size_t i = 0; i < count; i++) {
        data[i] -= mean;
    }
}

void algo_rectify_signal(float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return;
    }
    
    for (size_t i = 0; i < count; i++) {
        data[i] = fabsf(data[i]);
    }
}

void algo_integrate_accel_to_vel(const float *accel, float *velocity, 
                                 size_t count, float dt, float scale_factor)
{
    if (accel == NULL || velocity == NULL || count == 0 || dt <= 0.0f) {
        return;
    }
    
    // Trapezoidal integration
    velocity[0] = 0.0f;
    
    for (size_t i = 1; i < count; i++) {
        // Trapezoidal rule: ∫f(x)dx ≈ (f[i-1] + f[i]) * dt / 2
        float area = (accel[i-1] + accel[i]) * dt * 0.5f;
        velocity[i] = velocity[i-1] + area * scale_factor;
    }
}

void algo_calc_psd(const float *magnitude, float *psd, size_t n_fft, float fs)
{
    if (magnitude == NULL || psd == NULL || n_fft == 0 || fs <= 0.0f) {
        return;
    }
    
    size_t n_bins = n_fft / 2;
    float scale = 2.0f / (fs * n_fft); // PSD scaling factor
    
    for (size_t i = 0; i < n_bins; i++) {
        // Convert magnitude squared to PSD
        psd[i] = magnitude[i] * magnitude[i] * scale;
        
        // Apply correction for single-sided spectrum
        if (i > 0 && i < n_bins - 1) {
            psd[i] *= 2.0f;
        }
    }
}

/* ========================================================================= *
 * STATISTICAL UTILITIES IMPLEMENTATION
 * ========================================================================= */

float algo_calc_mean(const float *data, size_t count)
{
    if (data == NULL || count == 0) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += data[i];
    }
    
    return sum / count;
}

float algo_calc_variance(const float *data, size_t count, float mean)
{
    if (data == NULL || count <= 1) {
        return 0.0f;
    }
    
    float sum_sq_diff = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float diff = data[i] - mean;
        sum_sq_diff += diff * diff;
    }
    
    return sum_sq_diff / (count - 1); // Sample variance
}

float algo_calc_std_dev(const float *data, size_t count, float mean)
{
    float variance = algo_calc_variance(data, count, mean);
    return sqrtf(variance);
}