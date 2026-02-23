#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdint.h>
#include <math.h>
#include "icm42688p_baseline.h" // For icm_freq_profile_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t count;
    double m2_x, m2_y, m2_z; // Sum of Squared Differences
    double mean_x, mean_y, mean_z;
} algo_welford_t;

void algo_welford_init(algo_welford_t *ctx);
void algo_welford_update(algo_welford_t *ctx, float x, float y, float z);

// Calculate final RMS and correct with baseline (Delta = Measured - Baseline_Offset)
void algo_welford_finish(const algo_welford_t *ctx, const icm_freq_profile_t *baseline, 
                         float *out_x, float *out_y, float *out_z);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H