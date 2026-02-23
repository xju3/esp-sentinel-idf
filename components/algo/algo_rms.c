#include "algo_rms.h"
#include <string.h>
#include <math.h>

void algo_welford_init(algo_welford_t *ctx) {
    if (ctx) memset(ctx, 0, sizeof(algo_welford_t));
}

void algo_welford_update(algo_welford_t *ctx, float x, float y, float z) {
    if (!ctx) return;
    ctx->count++;
    
    // X Axis
    double delta_x = x - ctx->mean_x;
    ctx->mean_x += delta_x / ctx->count;
    ctx->m2_x += delta_x * (x - ctx->mean_x);

    // Y Axis
    double delta_y = y - ctx->mean_y;
    ctx->mean_y += delta_y / ctx->count;
    ctx->m2_y += delta_y * (y - ctx->mean_y);

    // Z Axis
    double delta_z = z - ctx->mean_z;
    ctx->mean_z += delta_z / ctx->count;
    ctx->m2_z += delta_z * (z - ctx->mean_z);
}

void algo_welford_finish(const algo_welford_t *ctx, const icm_freq_profile_t *baseline, 
                         float *out_x, float *out_y, float *out_z) {
    if (!ctx || !out_x || !out_y || !out_z) return;

    if (ctx->count < 2) {
        *out_x = *out_y = *out_z = 0.0f;
        return;
    }

    // 1. Calculate Standard Deviation (AC RMS)
    // Variance = M2 / (n-1)
    float raw_rms_x = sqrt(ctx->m2_x / (ctx->count - 1));
    float raw_rms_y = sqrt(ctx->m2_y / (ctx->count - 1));
    float raw_rms_z = sqrt(ctx->m2_z / (ctx->count - 1));

    // 2. Subtract Baseline Offset to get Delta
    if (baseline) {
        *out_x = (raw_rms_x > baseline->x.offset) ? (raw_rms_x - baseline->x.offset) : 0.0f;
        *out_y = (raw_rms_y > baseline->y.offset) ? (raw_rms_y - baseline->y.offset) : 0.0f;
        *out_z = (raw_rms_z > baseline->z.offset) ? (raw_rms_z - baseline->z.offset) : 0.0f;
    } else {
        *out_x = raw_rms_x;
        *out_y = raw_rms_y;
        *out_z = raw_rms_z;
    }
}