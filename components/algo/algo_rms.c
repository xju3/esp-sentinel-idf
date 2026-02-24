#include "algo_rms.h"
#include <string.h>
#include <math.h>
#include "logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern bool ici_42688_filter_initialized = false;
// ------------------------------------------------------------
// Welford RMS (existing)
// ------------------------------------------------------------
void algo_welford_init(algo_welford_t *ctx)
{
    if (ctx)
        memset(ctx, 0, sizeof(algo_welford_t));
}

void algo_welford_update(algo_welford_t *ctx, float x, float y, float z)
{
    if (!ctx)
        return;
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
                         float *out_x, float *out_y, float *out_z)
{
    if (!ctx || !out_x || !out_y || !out_z)
        return;

    if (ctx->count == 0)
    {
        *out_x = *out_y = *out_z = 0.0f;
        return;
    }

    /*
     * RMS relative to baseline = sqrt( Var(X) + (mean - baseline_mean)^2 ) - baseline_offset
     * - Var(X) is tracked online by Welford (m2 / (n-1))
     * - baseline_mean removes the DC component learned at boot
     * - baseline_offset (stddev during baseline) is treated as allowable noise floor
     */

    // Sample variance (protect count==1 to avoid div/0)
    float var_x = (ctx->count > 1) ? (float)(ctx->m2_x / (ctx->count - 1)) : 0.0f;
    float var_y = (ctx->count > 1) ? (float)(ctx->m2_y / (ctx->count - 1)) : 0.0f;
    float var_z = (ctx->count > 1) ? (float)(ctx->m2_z / (ctx->count - 1)) : 0.0f;

    // Baseline means (learned during boot). If absent, assume 0.
    float base_mean_x = baseline ? baseline->x.val : 0.0f;
    float base_mean_y = baseline ? baseline->y.val : 0.0f;
    float base_mean_z = baseline ? baseline->z.val : 0.0f;

    // RMS of baseline-removed signal
    float rms_x = sqrtf(fmaxf(0.0f, var_x + (float)pow(ctx->mean_x - base_mean_x, 2)));
    float rms_y = sqrtf(fmaxf(0.0f, var_y + (float)pow(ctx->mean_y - base_mean_y, 2)));
    float rms_z = sqrtf(fmaxf(0.0f, var_z + (float)pow(ctx->mean_z - base_mean_z, 2)));

    // Remove baseline noise floor (offset). Clamp at zero to avoid negatives.
    if (baseline)
    {
        *out_x = fmaxf(0.0f, rms_x - baseline->x.offset);
        *out_y = fmaxf(0.0f, rms_y - baseline->y.offset);
        *out_z = fmaxf(0.0f, rms_z - baseline->z.offset);
    }
    else
    {
        *out_x = rms_x;
        *out_y = rms_y;
        *out_z = rms_z;
    }
}

// ------------------------------------------------------------
// ISO10816/20816 velocity RMS calculation
// ------------------------------------------------------------

// Compute biquad coefficients (RBJ cookbook) for HP/LP Butterworth order=2
// type: 0 = lowpass, 1 = highpass
static void biquad_design(algo_biquad_t *bq, int type, float fc, float fs)
{
    // protect
    if (!bq || fs <= 0 || fc <= 0)
        return;

    float w0 = 2.0f * (float)M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float Q = 1.0f / sqrtf(2.0f); // Butterworth
    float alpha = sinw0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;
    if (type == 0)
    { // LP
        b0 = (1 - cosw0) * 0.5f;
        b1 = 1 - cosw0;
        b2 = (1 - cosw0) * 0.5f;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
    }
    else
    { // HP
        b0 = (1 + cosw0) * 0.5f;
        b1 = -(1 + cosw0);
        b2 = (1 + cosw0) * 0.5f;
        a0 = 1 + alpha;
        a1 = -2 * cosw0;
        a2 = 1 - alpha;
    }

    // Normalize
    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;
    bq->z1 = bq->z2 = 0.0f; // reset state on redesign
}

static inline float biquad_process(algo_biquad_t *bq, float x)
{
    float y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}

void iso10816_init(iso10816_state_t *st)
{
    
    if (!st)
        return;
    memset(st, 0, sizeof(*st));
}

void iso10816_compute(iso10816_state_t *st, const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out)
{
    st->vx = st->vy = st->vz = 0.0f;
    if (!st || !ax || !ay || !az || !out || N <= 0 || fs <= 0)
    {
        LOG_INFOF("ISO DBG: early return st=%p ax=%p ... N=%d fs=%.1f", st, ax, N, fs);
        return;
    }
    // Recompute coefficients if fs changed (or first time)
    if (fabsf(st->fs - fs) > 5.0f)
    {
        float fc_hp10 = 10.0f;
        float fc_lp = fminf(1000.0f, 0.45f * fs);
        float fc_hp1 = 1.0f;

        biquad_design(&st->hp10_x, 1, fc_hp10, fs);
        biquad_design(&st->hp10_y, 1, fc_hp10, fs);
        biquad_design(&st->hp10_z, 1, fc_hp10, fs);

        biquad_design(&st->lp_x, 0, fc_lp, fs);
        biquad_design(&st->lp_y, 0, fc_lp, fs);
        biquad_design(&st->lp_z, 0, fc_lp, fs);

        biquad_design(&st->hp1_vx, 1, fc_hp1, fs);
        biquad_design(&st->hp1_vy, 1, fc_hp1, fs);
        biquad_design(&st->hp1_vz, 1, fc_hp1, fs);

        st->fs = fs;
    }

    const float g_to_ms2 = 9.80665f;
    float sum_vx2 = 0, sum_vy2 = 0, sum_vz2 = 0;
    float dt = 1.0f / fs;
    for (int i = 0; i < N; ++i)
    {
        // g -> m/s^2
        float x = ax[i] * g_to_ms2;
        float y = ay[i] * g_to_ms2;
        float z = az[i] * g_to_ms2;

        // Band-pass accel 10 Hz HP + LP fc_lp
        x = biquad_process(&st->hp10_x, x);
        y = biquad_process(&st->hp10_y, y);
        z = biquad_process(&st->hp10_z, z);

        x = biquad_process(&st->lp_x, x);
        y = biquad_process(&st->lp_y, y);
        z = biquad_process(&st->lp_z, z);

        // Integrate to velocity (trapezoidal could be used; here: simple rectangular)
        st->vx += x * dt;
        st->vy += y * dt;
        st->vz += z * dt;

        // High-pass velocity at 1 Hz to remove drift
        float vx_hp = biquad_process(&st->hp1_vx, st->vx);
        float vy_hp = biquad_process(&st->hp1_vy, st->vy);
        float vz_hp = biquad_process(&st->hp1_vz, st->vz);

        sum_vx2 += vx_hp * vx_hp;
        sum_vy2 += vy_hp * vy_hp;
        sum_vz2 += vz_hp * vz_hp;
    }

    float invN = 1.0f / (float)N;
    out->vx_rms = sqrtf(sum_vx2 * invN) * 1000.0f; // m/s -> mm/s
    out->vy_rms = sqrtf(sum_vy2 * invN) * 1000.0f;
    out->vz_rms = sqrtf(sum_vz2 * invN) * 1000.0f;

    float v3d = sum_vx2 + sum_vy2 + sum_vz2;
    out->v3d_rms = sqrtf(v3d * invN) * 1000.0f;
}
