// algo_rms.c
// Robust ISO10816-style velocity RMS (mm/s RMS) from accel (g) input.
// Band: 10 Hz HP + LP (min(1000, 0.45*fs)) on acceleration, integrate to velocity,
// then 1 Hz HP on velocity to suppress residual drift.
// Key robustness features:
//  - Separate biquad coefficient design from state reset.
//  - Avoid redesign on tiny fs jitter (tolerance).
//  - Skip initial samples to avoid IIR cold-start transient contaminating RMS.
//  - Safe pointer checks before dereference.
#include "algo_test.h"
#include "algo_rms.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>



// ----------------------------- Internals -----------------------------

static inline void t_biquad_reset_state(biquad_t *q) {
    q->s1 = 0.0f;
    q->s2 = 0.0f;
}

static inline float t_biquad_process(biquad_t *q, float x) {
    // Direct Form II Transposed (DF2T)
    // y = b0*x + s1
    // s1 = b1*x - a1*y + s2
    // s2 = b2*x - a2*y
    float y = q->b0 * x + q->s1;
    float s1_new = q->b1 * x - q->a1 * y + q->s2;
    float s2_new = q->b2 * x - q->a2 * y;
    q->s1 = s1_new;
    q->s2 = s2_new;
    return y;
}

// 2nd-order Butterworth biquad design using RBJ cookbook-style formulas,
// with bilinear transform via tan() pre-warp. Stable and common for embedded.
// type: 0 = lowpass, 1 = highpass
static void t_biquad_design_butter2(biquad_t *q, int type, float fc, float fs) {
    // Guard against nonsense
    if (fs <= 0.0f) fs = 1.0f;

    // Clamp cutoff to (0, Nyquist)
    // float nyq = 0.5f * fs;
    if (fc < 0.001f) fc = 0.001f;
    if (fc > 0.499f * fs) fc = 0.499f * fs;

    // Butterworth Q for 2nd-order section
    const float Q = 0.70710678118f;

    // Pre-warp
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    if (type == 0) {
        // Low-pass
        b0 = (1.0f - cosw0) * 0.5f;
        b1 = 1.0f - cosw0;
        b2 = (1.0f - cosw0) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;
    } else {
        // High-pass
        b0 = (1.0f + cosw0) * 0.5f;
        b1 = -(1.0f + cosw0);
        b2 = (1.0f + cosw0) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;
    }

    // Normalize (a0 = 1)
    float inv_a0 = 1.0f / a0;
    q->b0 = b0 * inv_a0;
    q->b1 = b1 * inv_a0;
    q->b2 = b2 * inv_a0;
    q->a1 = a1 * inv_a0;
    q->a2 = a2 * inv_a0;

    // IMPORTANT: do NOT reset state here.
    // State reset should be a separate, explicit choice.
}

void t_iso10816_init(t_iso10816_state_t *st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->fs_last = 0.0f;
}

// Helper: (Re)design filters if needed, with tolerance to avoid churn.
static void t_iso10816_maybe_design(t_iso10816_state_t *st, float fs) {
    // If you pass fixed ODR, this will run only once.
    const float FS_TOL_HZ = 50.0f; // tolerate jitter; adjust as you like

    if (st->fs_last > 0.0f && fabsf(st->fs_last - fs) <= FS_TOL_HZ) {
        return; // close enough; keep existing coefficients and states
    }

    // Design parameters
    const float fc_hp10 = 10.0f;
    const float fc_hp1  = 1.0f;

    float fc_lp = 500.0f;
    // float fc_lp = 1000.0f;
    float max_lp = 0.45f * fs;
    if (fc_lp > max_lp) fc_lp = max_lp;
    if (fc_lp < 50.0f)  fc_lp = max_lp; // if fs is low, just use near-Nyquist LP

    // Design coefficients (no state reset here)
    t_biquad_design_butter2(&st->hp10_x, 1, fc_hp10, fs);
    t_biquad_design_butter2(&st->hp10_y, 1, fc_hp10, fs);
    t_biquad_design_butter2(&st->hp10_z, 1, fc_hp10, fs);

    t_biquad_design_butter2(&st->lp_x,   0, fc_lp, fs);
    t_biquad_design_butter2(&st->lp_y,   0, fc_lp, fs);
    t_biquad_design_butter2(&st->lp_z,   0, fc_lp, fs);

    t_biquad_design_butter2(&st->hp1_vx, 1, fc_hp1, fs);
    t_biquad_design_butter2(&st->hp1_vy, 1, fc_hp1, fs);
    t_biquad_design_butter2(&st->hp1_vz, 1, fc_hp1, fs);

    // Since coefficients changed a lot, it's usually safest to reset states here.
    // BUT: if you want continuous streaming behavior, you can omit resets.
    t_biquad_reset_state(&st->hp10_x); t_biquad_reset_state(&st->hp10_y); t_biquad_reset_state(&st->hp10_z);
    t_biquad_reset_state(&st->lp_x);   t_biquad_reset_state(&st->lp_y);   t_biquad_reset_state(&st->lp_z);
    t_biquad_reset_state(&st->hp1_vx); t_biquad_reset_state(&st->hp1_vy); t_biquad_reset_state(&st->hp1_vz);

    st->vx = st->vy = st->vz = 0.0f; // reset integrator on redesign

    st->fs_last = fs;
}

void t_iso10816_compute(t_iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out)
{
    if (!st || !ax || !ay || !az || !out || N <= 0 || fs <= 0.0f) {
        return;
    }

    // Ensure filters are designed (tolerant to fs jitter).
    t_iso10816_maybe_design(st, fs);

    // Window behavior: per-window independent measure
    // Reset integrator each window (recommended for your "every minute 1-second snapshot" design).
    st->vx = st->vy = st->vz = 0.0f;

    const float dt = 1.0f / fs;
    const float G2MPS2 = 9.80665f;

    // Skip initial samples to avoid IIR cold-start transient contaminating RMS.
    // Good defaults: 256~1024 for 4kHz, depending on how strict you want.
    int skip = 1024;
    if (skip > N) skip = N;

    double sum_vx2 = 0.0, sum_vy2 = 0.0, sum_vz2 = 0.0;
    int used = 0;

    for (int i = 0; i < N; i++) {
        // Convert to m/s^2
        float x = ax[i] * G2MPS2;
        float y = ay[i] * G2MPS2;
        float z = az[i] * G2MPS2;

        // Accel bandpass: HP10 -> LP
        x = t_biquad_process(&st->hp10_x, x);
        y = t_biquad_process(&st->hp10_y, y);
        z = t_biquad_process(&st->hp10_z, z);

        x = t_biquad_process(&st->lp_x, x);
        y = t_biquad_process(&st->lp_y, y);
        z = t_biquad_process(&st->lp_z, z);

        // Integrate to velocity (m/s)
        st->vx += x * dt;
        st->vy += y * dt;
        st->vz += z * dt;

        // Remove drift on velocity
        float vx_hp = t_biquad_process(&st->hp1_vx, st->vx);
        float vy_hp = t_biquad_process(&st->hp1_vy, st->vy);
        float vz_hp = t_biquad_process(&st->hp1_vz, st->vz);

        // After filters have settled, accumulate RMS
        if (i >= skip) {
            float vx_mmps = vx_hp * 1000.0f;
            float vy_mmps = vy_hp * 1000.0f;
            float vz_mmps = vz_hp * 1000.0f;

            sum_vx2 += (double)vx_mmps * vx_mmps;
            sum_vy2 += (double)vy_mmps * vy_mmps;
            sum_vz2 += (double)vz_mmps * vz_mmps;
            used++;
        }
    }

    if (used <= 0) {
        out->vx_rms = out->vy_rms = out->vz_rms = out->v3d_rms = 0.0f;
        return;
    }

    float vx_rms = (float)sqrt(sum_vx2 / used);
    float vy_rms = (float)sqrt(sum_vy2 / used);
    float vz_rms = (float)sqrt(sum_vz2 / used);

    out->vx_rms = vx_rms;
    out->vy_rms = vy_rms;
    out->vz_rms = vz_rms;
    out->v3d_rms = sqrtf(vx_rms*vx_rms + vy_rms*vy_rms + vz_rms*vz_rms);
}