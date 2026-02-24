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

typedef struct {
    // Direct Form II transposed biquad
    float b0, b1, b2;
    float a1, a2;
    float z1, z2; // state
} algo_biquad_t;

typedef struct {
    float vx_rms;
    float vy_rms;
    float vz_rms;
    float v3d_rms;
} iso10816_result_t;

typedef struct {
    algo_biquad_t hp10_x, hp10_y, hp10_z;   // 10 Hz HP on acceleration
    algo_biquad_t lp_x, lp_y, lp_z;         // LP at fc_lp on acceleration
    algo_biquad_t hp1_vx, hp1_vy, hp1_vz;   // 1 Hz HP on velocity
    float vx, vy, vz;                       // integrator states (m/s)
    float fs;                               // last fs used to derive coeffs
} iso10816_state_t;

void algo_welford_init(algo_welford_t *ctx);
void algo_welford_update(algo_welford_t *ctx, float x, float y, float z);

// Calculate final RMS and correct with baseline (Delta = Measured - Baseline_Offset)
void algo_welford_finish(const algo_welford_t *ctx, const icm_freq_profile_t *baseline, 
                         float *out_x, float *out_y, float *out_z);

// Initialize ISO10816 state (resets filter states & velocity integrators)
void iso10816_init(iso10816_state_t *st);

// Compute vibration velocity (mm/s RMS) per ISO10816/20816 band (10-1000 Hz) from acceleration in g.
// fs: sampling rate in Hz, ax/ay/az length N.
void iso10816_compute(iso10816_state_t *st, const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H
