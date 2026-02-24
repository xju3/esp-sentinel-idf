#ifndef ALGO_TEST_H
#define ALGO_TEST_H

#include <stdint.h>
#include "algo_rms.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    // biquad coefficients (normalized a0 = 1)
    float b0, b1, b2;
    float a1, a2;
    // DF2T state
    float s1, s2;
} biquad_t;


typedef struct {
    float fs_last;

    // Accel bandpass = HP10 + LP(fc_lp)
    biquad_t hp10_x, hp10_y, hp10_z;
    biquad_t lp_x,   lp_y,   lp_z;

    // Velocity high-pass to remove drift
    biquad_t hp1_vx, hp1_vy, hp1_vz;

    // integrator state (velocity in m/s)
    float vx, vy, vz;
} t_iso10816_state_t;


// Initialize state (safe default)
void t_iso10816_init(t_iso10816_state_t *st);

// Compute velocity RMS for one window.
// ax/ay/az are arrays in g.
// N samples, fs in Hz.
// out filled with RMS results.
void t_iso10816_compute(t_iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // ALGO_TEST_H
