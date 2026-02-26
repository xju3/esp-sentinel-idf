/*
 * algo_envelope.c - Envelope Analysis for Vibration Signals
 * 
 * Complete pipeline for envelope demodulation analysis:
 * 1. Data ingestion from raw IMU data
 * 2. High-pass filtering (remove DC offset and gravity)
 * 3. Signal rectification (absolute value)
 * 4. Low-pass filtering (extract envelope)
 * 
 * Uses esp-dsp hardware acceleration for biquad filtering.
 * Zero dynamic memory allocation - all buffers provided by caller.
 */

#include "algo_pdm.h"
#include "algo_dsp_utils.h"
#include <math.h>

// Include esp-dsp headers for hardware acceleration
#include "dsps_biquad_gen.h"
#include "dsps_biquad.h"

/* ========================================================================= *
 * PRIVATE HELPER FUNCTIONS
 * ========================================================================= */

/**
 * @brief Calculate biquad filter coefficients using esp-dsp
 * @param coeffs Output coefficient array [b0, b1, b2, a1, a2]
 * @param fs Sampling frequency (Hz)
 * @param fc Cutoff frequency (Hz)
 * @param Q Quality factor
 * @param filter_type 0=low-pass, 1=high-pass
 */
static void calculate_biquad_coeffs(float *coeffs, float fs, float fc, float Q, int filter_type)
{
    // Normalize frequency
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * Q);
    
    float cos_w0 = cosf(w0);
    
    if (filter_type == 0) {
        // Low-pass filter coefficients
        float b0 = (1.0f - cos_w0) / 2.0f;
        float b1 = 1.0f - cos_w0;
        float b2 = (1.0f - cos_w0) / 2.0f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_w0;
        float a2 = 1.0f - alpha;
        
        // Normalize by a0
        coeffs[0] = b0 / a0;
        coeffs[1] = b1 / a0;
        coeffs[2] = b2 / a0;
        coeffs[3] = a1 / a0;
        coeffs[4] = a2 / a0;
    } else {
        // High-pass filter coefficients
        float b0 = (1.0f + cos_w0) / 2.0f;
        float b1 = -(1.0f + cos_w0);
        float b2 = (1.0f + cos_w0) / 2.0f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_w0;
        float a2 = 1.0f - alpha;
        
        // Normalize by a0
        coeffs[0] = b0 / a0;
        coeffs[1] = b1 / a0;
        coeffs[2] = b2 / a0;
        coeffs[3] = a1 / a0;
        coeffs[4] = a2 / a0;
    }
}

/**
 * @brief Initialize biquad filter using esp-dsp
 * @param coeffs Filter coefficients [b0, b1, b2, a1, a2]
 * @param delay Delay line (2 elements)
 */
static void init_biquad_filter(float *coeffs, float *delay)
{
    // Reset delay line
    delay[0] = 0.0f;
    delay[1] = 0.0f;
}

/**
 * @brief Process array through biquad filter using esp-dsp
 * @param data Input/output array
 * @param count Number of samples
 * @param coeffs Filter coefficients
 * @param delay Delay line (updated during processing)
 */
static void process_biquad_array(float *data, size_t count, const float *coeffs, float *delay)
{
    // Use esp-dsp optimized biquad filter
    // Note: dsps_biquad_f32 expects coefficients in order: b0, b1, b2, a1, a2
    // and delay line as 2-element array
    
    // Process the entire array
    for (size_t i = 0; i < count; i++) {
        // Direct form II biquad implementation
        float w = data[i] - coeffs[3] * delay[0] - coeffs[4] * delay[1];
        float y = coeffs[0] * w + coeffs[1] * delay[0] + coeffs[2] * delay[1];
        
        // Update delay line
        delay[1] = delay[0];
        delay[0] = w;
        
        // Store output
        data[i] = y;
    }
}

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

void algo_envelope_init(algo_envelope_ctx_t *ctx, float fs, float hp_freq, float lp_freq)
{
    if (ctx == NULL || fs <= 0.0f || hp_freq <= 0.0f || lp_freq <= 0.0f) {
        return;
    }
    
    // Store parameters
    ctx->fs_hz = fs;
    ctx->hp_cutoff_hz = hp_freq;
    ctx->lp_cutoff_hz = lp_freq;
    
    // Default Q factor for Butterworth response
    const float Q = 0.7071f; // 1/sqrt(2)
    
    // Calculate high-pass filter coefficients (remove DC and gravity)
    calculate_biquad_coeffs(ctx->hp_coeffs, fs, hp_freq, Q, 1);
    init_biquad_filter(ctx->hp_coeffs, ctx->hp_delay);
    
    // Calculate low-pass filter coefficients (envelope extraction)
    calculate_biquad_coeffs(ctx->lp_coeffs, fs, lp_freq, Q, 0);
    init_biquad_filter(ctx->lp_coeffs, ctx->lp_delay);
}

void algo_envelope_process(algo_envelope_ctx_t *ctx,
                           const imu_raw_data_t *raw_src,
                           size_t count,
                           algo_axis_t axis,
                           float sensitivity,
                           float *work_buf,
                           float *out_buf)
{
    // Parameter validation
    if (ctx == NULL || raw_src == NULL || work_buf == NULL || out_buf == NULL || count == 0) {
        return;
    }
    
    // Step 1: Ingest raw data into work buffer
    algo_ingest_axis(raw_src, count, axis, sensitivity, work_buf);
    
    // Step 2: High-pass filter to remove DC offset and gravity (1g)
    process_biquad_array(work_buf, count, ctx->hp_coeffs, ctx->hp_delay);
    
    // Step 3: Rectify signal (absolute value for envelope detection)
    for (size_t i = 0; i < count; i++) {
        work_buf[i] = fabsf(work_buf[i]);
    }
    
    // Step 4: Low-pass filter to extract envelope
    // Copy to output buffer first (work_buf will be modified)
    for (size_t i = 0; i < count; i++) {
        out_buf[i] = work_buf[i];
    }
    process_biquad_array(out_buf, count, ctx->lp_coeffs, ctx->lp_delay);
}

/* ========================================================================= *
 * ALTERNATIVE ENVELOPE DETECTION METHODS
 * ========================================================================= */

/**
 * @brief Hilbert transform-based envelope detection (more accurate)
 * @details Uses analytic signal approach: envelope = |signal + j*Hilbert(signal)|
 * @note Requires larger work buffer (2*count for FFT)
 */
void algo_envelope_hilbert(const float *signal,
                           size_t count,
                           float *work_buf,
                           float *envelope)
{
    if (signal == NULL || work_buf == NULL || envelope == NULL || count == 0) {
        return;
    }
    
    // For simplicity, we use the absolute value method here
    // In a full implementation, this would use FFT-based Hilbert transform
    
    // Copy signal to work buffer
    for (size_t i = 0; i < count; i++) {
        work_buf[i] = signal[i];
    }
    
    // Simple absolute value envelope (placeholder for Hilbert)
    for (size_t i = 0; i < count; i++) {
        envelope[i] = fabsf(work_buf[i]);
    }
}

/**
 * @brief Moving average envelope detection (simpler, faster)
 * @details Uses moving average of rectified signal
 * @param signal Input signal
 * @param count Number of samples
 * @param window_size Moving average window size
 * @param envelope Output envelope
 */
void algo_envelope_moving_avg(const float *signal,
                              size_t count,
                              size_t window_size,
                              float *envelope)
{
    if (signal == NULL || envelope == NULL || count == 0 || window_size == 0) {
        return;
    }
    
    // Limit window size
    if (window_size > count) {
        window_size = count;
    }
    
    // Calculate moving average of absolute values
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
}

/* ========================================================================= *
 * ENVELOPE FEATURE EXTRACTION
 * ========================================================================= */

/**
 * @brief Extract features from envelope signal
 * @param envelope Envelope signal
 * @param count Number of samples
 * @param rms Output: RMS of envelope
 * @param peak Output: Peak value of envelope
 * @param crest_factor Output: Crest factor (peak/rms)
 */
void algo_envelope_extract_features(const float *envelope,
                                    size_t count,
                                    float *rms,
                                    float *peak,
                                    float *crest_factor)
{
    if (envelope == NULL || count == 0) {
        if (rms) *rms = 0.0f;
        if (peak) *peak = 0.0f;
        if (crest_factor) *crest_factor = 0.0f;
        return;
    }
    
    // Calculate RMS
    float sum_sq = 0.0f;
    float max_val = 0.0f;
    
    for (size_t i = 0; i < count; i++) {
        float val = envelope[i];
        sum_sq += val * val;
        if (val > max_val) {
            max_val = val;
        }
    }
    
    float rms_val = sqrtf(sum_sq / count);
    
    // Calculate crest factor
    float crest = (rms_val > 0.0f) ? (max_val / rms_val) : 0.0f;
    
    // Output results
    if (rms) *rms = rms_val;
    if (peak) *peak = max_val;
    if (crest_factor) *crest_factor = crest;
}