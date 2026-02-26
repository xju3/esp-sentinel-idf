/*
 * algo_welford.c - Welford's Online Algorithm for Streaming Statistics
 * 
 * O(1) space complexity algorithm for computing mean and variance incrementally.
 * Processes raw IMU data directly without intermediate float arrays.
 */

#include "algo_pdm.h"
#include "algo_dsp_utils.h"
#include <math.h>

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

void algo_welford_update(algo_welford_ctx_t *ctx, 
                         const imu_raw_data_t *src, 
                         size_t count, 
                         algo_axis_t axis, 
                         float sensitivity)
{
    if (ctx == NULL || src == NULL || count == 0) {
        return;
    }
    
    // Convert enum to integer axis index
    int axis_idx = (int)axis;
    if (axis_idx < 0 || axis_idx > 2) {
        axis_idx = 2; // Default to Z-axis
    }
    
    // Process each sample
    for (size_t i = 0; i < count; i++) {
        // Extract and convert raw value
        int16_t raw_val = algo_extract_axis_raw(&src[i], axis_idx);
        float sample = algo_scale_raw_to_float(raw_val, sensitivity);
        
        // Update min/max
        if (ctx->count == 0) {
            ctx->min_val = sample;
            ctx->max_val = sample;
        } else {
            if (sample < ctx->min_val) ctx->min_val = sample;
            if (sample > ctx->max_val) ctx->max_val = sample;
        }
        
        // Welford's algorithm for online mean and variance
        ctx->count++;
        
        double delta = sample - ctx->mean;
        ctx->mean += delta / ctx->count;
        double delta2 = sample - ctx->mean;
        ctx->m2 += delta * delta2;
    }
}

void algo_welford_get_stats(const algo_welford_ctx_t *ctx,
                            float *mean, float *variance, float *std_dev)
{
    if (ctx == NULL) {
        if (mean) *mean = 0.0f;
        if (variance) *variance = 0.0f;
        if (std_dev) *std_dev = 0.0f;
        return;
    }
    
    if (ctx->count == 0) {
        if (mean) *mean = 0.0f;
        if (variance) *variance = 0.0f;
        if (std_dev) *std_dev = 0.0f;
        return;
    }
    
    if (mean) *mean = (float)ctx->mean;
    
    if (variance || std_dev) {
        double var = (ctx->count > 1) ? (ctx->m2 / (ctx->count - 1)) : 0.0;
        if (variance) *variance = (float)var;
        if (std_dev) *std_dev = (float)sqrt(var);
    }
}

/* ========================================================================= *
 * HELPER FUNCTIONS
 * ========================================================================= */

/**
 * @brief Reset Welford context to initial state
 */
void algo_welford_reset(algo_welford_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    
    ctx->count = 0;
    ctx->mean = 0.0;
    ctx->m2 = 0.0;
    ctx->min_val = 0.0f;
    ctx->max_val = 0.0f;
}

/**
 * @brief Combine statistics from multiple Welford contexts
 * @details Useful for parallel processing or distributed computation
 */
void algo_welford_combine(algo_welford_ctx_t *result,
                          const algo_welford_ctx_t *a,
                          const algo_welford_ctx_t *b)
{
    if (result == NULL || a == NULL || b == NULL) {
        return;
    }
    
    if (a->count == 0) {
        *result = *b;
        return;
    }
    
    if (b->count == 0) {
        *result = *a;
        return;
    }
    
    // Combine counts
    uint32_t total_count = a->count + b->count;
    
    // Combine means using Chan's formula
    double delta = b->mean - a->mean;
    double mean = a->mean + delta * b->count / total_count;
    
    // Combine M2 using Chan's formula
    double m2 = a->m2 + b->m2 + delta * delta * a->count * b->count / total_count;
    
    // Combine min/max
    float min_val = (a->min_val < b->min_val) ? a->min_val : b->min_val;
    float max_val = (a->max_val > b->max_val) ? a->max_val : b->max_val;
    
    // Store results
    result->count = total_count;
    result->mean = mean;
    result->m2 = m2;
    result->min_val = min_val;
    result->max_val = max_val;
}

/**
 * @brief Get current statistics as individual values (for debugging)
 * @details Provides all statistics as separate output parameters
 */
void algo_welford_get_all_stats(const algo_welford_ctx_t *ctx,
                                uint32_t *count,
                                float *mean,
                                float *std_dev,
                                float *min_val,
                                float *max_val)
{
    if (ctx == NULL) {
        if (count) *count = 0;
        if (mean) *mean = 0.0f;
        if (std_dev) *std_dev = 0.0f;
        if (min_val) *min_val = 0.0f;
        if (max_val) *max_val = 0.0f;
        return;
    }
    
    if (count) *count = ctx->count;
    if (min_val) *min_val = ctx->min_val;
    if (max_val) *max_val = ctx->max_val;
    
    if (ctx->count == 0) {
        if (mean) *mean = 0.0f;
        if (std_dev) *std_dev = 0.0f;
        return;
    }
    
    if (mean) *mean = (float)ctx->mean;
    
    if (std_dev) {
        double var = (ctx->count > 1) ? (ctx->m2 / (ctx->count - 1)) : 0.0;
        *std_dev = (float)sqrt(var);
    }
}
