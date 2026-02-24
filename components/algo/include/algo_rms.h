// ============================================================
// algo_rms.h + algo_rms.c (merged)
//
// Fixes/Improvements vs previous version:
//  - Safe NULL checks before dereferencing pointers
//  - Biquad design no longer resets state implicitly; reset is explicit
//  - Avoid frequent redesign on small fs jitter (tolerance)
//  - Skip initial samples to avoid IIR cold-start transient contaminating RMS
//  - Uses DF2T biquad (same structure as your code) with explicit state
//  - Keeps your Welford baseline-corrected accel-RMS path unchanged
//
// Notes:
//  - ISO10816/20816 commonly uses vibration VELOCITY RMS (mm/s RMS).
//  - This implementation computes per-axis velocity RMS over a band:
//      accel: 10 Hz HP + LP(fc_lp), integrate to velocity, then 1 Hz HP.
//  - For your “every minute capture 1 second” workflow, we reset integrators
//    each call, and skip the first part of the window so the IIR settles.
// ============================================================

#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "icm42688p_baseline.h" // icm_freq_profile_t

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- Welford ----------------

typedef struct {
    uint32_t count;
    double m2_x, m2_y, m2_z;
    double mean_x, mean_y, mean_z;
} algo_welford_t;

void algo_welford_init(algo_welford_t *ctx);
void algo_welford_update(algo_welford_t *ctx, float x, float y, float z);

// Baseline-corrected accel RMS feature (your existing logic):
// out = max(0, sqrt(var + (mean-base_mean)^2) - base_offset)
void algo_welford_finish(const algo_welford_t *ctx,
                         const icm_freq_profile_t *baseline,
                         float *out_x, float *out_y, float *out_z);

// ---------------- ISO10816/20816 velocity RMS ----------------

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float s1, s2; // DF2T state
} algo_biquad_t;

typedef struct {
    float vx_rms;
    float vy_rms;
    float vz_rms;
    float v3d_rms;
} iso10816_result_t;

typedef struct {
    // Accel bandpass: 10 Hz HP + LP(fc_lp)
    algo_biquad_t hp10_x, hp10_y, hp10_z;
    algo_biquad_t lp_x,   lp_y,   lp_z;

    // Velocity high-pass: 1 Hz HP
    algo_biquad_t hp1_vx, hp1_vy, hp1_vz;

    // Velocity integrators (m/s)
    float vx, vy, vz;

    // Last fs used for coefficient design
    float fs;

    // Tunables
    float fc_lp_hz;      // default 1000 Hz (or min(1000, 0.45*fs))
    float skip_seconds;  // default 0.25s
    float fs_tol_hz;     // default 50 Hz
} iso10816_state_t;

void iso10816_init(iso10816_state_t *st);

// Optional setters (can ignore if you want defaults)
void iso10816_set_fc_lp(iso10816_state_t *st, float fc_lp_hz);
void iso10816_set_skip_seconds(iso10816_state_t *st, float skip_seconds);
void iso10816_set_fs_tolerance(iso10816_state_t *st, float fs_tol_hz);

// Compute velocity RMS (mm/s RMS) for one window.
// ax/ay/az are acceleration in g.
// fs is sampling rate in Hz.
void iso10816_compute(iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H