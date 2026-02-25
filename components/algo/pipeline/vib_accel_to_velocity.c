// pipeline/vib_accel_to_velocity.c

#include "pipeline/vib_accel_to_velocity.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline void remove_mean_3axis(const float *ax, const float *ay, const float *az,
                                     int N, float *mx, float *my, float *mz)
{
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (int i = 0; i < N; ++i) {
        sx += ax[i];
        sy += ay[i];
        sz += az[i];
    }
    double invN = 1.0 / (double)N;
    *mx = (float)(sx * invN);
    *my = (float)(sy * invN);
    *mz = (float)(sz * invN);
}

static void design_if_needed(vib_accel_to_velocity_state_t *st, float fs_hz)
{
    float tol = (st->cfg.fs_tol_hz > 0.0f) ? st->cfg.fs_tol_hz : 50.0f;
    if (st->fs_last > 0.0f && fabsf(st->fs_last - fs_hz) <= tol) {
        return;
    }

    float hp_a = (st->cfg.hp_accel_hz > 0.0f) ? st->cfg.hp_accel_hz : 10.0f;
    float lp_a = (st->cfg.lp_accel_hz > 0.0f) ? st->cfg.lp_accel_hz : 1000.0f;
    float hp_v = (st->cfg.hp_vel_hz > 0.0f) ? st->cfg.hp_vel_hz : 1.0f;

    // LP clamp
    float max_lp = 0.45f * fs_hz;
    if (lp_a > max_lp) lp_a = max_lp;

    // 设计系数（design 不reset）
    (void)vib_biquad_design_butter2_hp(&st->hp10_x, hp_a, fs_hz);
    (void)vib_biquad_design_butter2_hp(&st->hp10_y, hp_a, fs_hz);
    (void)vib_biquad_design_butter2_hp(&st->hp10_z, hp_a, fs_hz);

    (void)vib_biquad_design_butter2_lp(&st->lp_x, lp_a, fs_hz);
    (void)vib_biquad_design_butter2_lp(&st->lp_y, lp_a, fs_hz);
    (void)vib_biquad_design_butter2_lp(&st->lp_z, lp_a, fs_hz);

    (void)vib_biquad_design_butter2_hp(&st->hp1_vx, hp_v, fs_hz);
    (void)vib_biquad_design_butter2_hp(&st->hp1_vy, hp_v, fs_hz);
    (void)vib_biquad_design_butter2_hp(&st->hp1_vz, hp_v, fs_hz);

    // 重新设计系数时，为避免“旧状态+新系数”失配污染，强制 reset
    vib_accel_to_velocity_reset_state(st);
    st->fs_last = fs_hz;
}

void vib_accel_to_velocity_init(vib_accel_to_velocity_state_t *st)
{
    if (!st) return;
    st->vx = st->vy = st->vz = 0.0f;
    st->fs_last = 0.0f;

    st->cfg.unit = VIB_ACCEL_UNIT_G;
    st->cfg.enable_bandpass = true;
    st->cfg.enable_remove_mean = true;
    st->cfg.hp_accel_hz = 10.0f;
    st->cfg.lp_accel_hz = 1000.0f;
    st->cfg.hp_vel_hz = 1.0f;
    st->cfg.skip_seconds = 0.25f;
    st->cfg.fs_tol_hz = 50.0f;
    st->cfg.leak_hz = 0.5f;

    vib_accel_to_velocity_reset_state(st);
}

vib_algo_err_t vib_accel_to_velocity_set_cfg(vib_accel_to_velocity_state_t *st,
                                            const vib_accel_to_velocity_cfg_t *cfg)
{
    if (!st || !cfg) return VIB_ALGO_BAD_ARGS;
    st->cfg = *cfg;
    return VIB_ALGO_OK;
}

void vib_accel_to_velocity_reset_state(vib_accel_to_velocity_state_t *st)
{
    if (!st) return;
    vib_biquad_reset(&st->hp10_x); vib_biquad_reset(&st->hp10_y); vib_biquad_reset(&st->hp10_z);
    vib_biquad_reset(&st->lp_x);   vib_biquad_reset(&st->lp_y);   vib_biquad_reset(&st->lp_z);
    vib_biquad_reset(&st->hp1_vx); vib_biquad_reset(&st->hp1_vy); vib_biquad_reset(&st->hp1_vz);
    st->vx = st->vy = st->vz = 0.0f;
}

vib_algo_err_t vib_accel_to_velocity_window(vib_accel_to_velocity_state_t *st,
                                           const float *ax, const float *ay, const float *az,
                                           int N, float fs_hz,
                                           float *vx_out, float *vy_out, float *vz_out,
                                           int *used_from)
{
    if (!st || !ax || !ay || !az || !vx_out || !vy_out || !vz_out) return VIB_ALGO_BAD_ARGS;
    if (N <= 0 || !(fs_hz > 0.0f)) return VIB_ALGO_BAD_ARGS;

    if (used_from) *used_from = 0;

    design_if_needed(st, fs_hz);

    const float dt = 1.0f / fs_hz;
    const float g_to_ms2 = 9.80665f;

    float leak = 1.0f;
    if (st->cfg.leak_hz > 0.0f) {
        leak = expf(-2.0f * (float)M_PI * st->cfg.leak_hz * dt);
    }

    int skip = 0;
    if (st->cfg.skip_seconds > 0.0f) {
        skip = (int)lroundf(st->cfg.skip_seconds * fs_hz);
        if (skip < 0) skip = 0;
        if (skip > N) skip = N;
    }
    if (used_from) *used_from = skip;

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    if (st->cfg.enable_remove_mean) {
        remove_mean_3axis(ax, ay, az, N, &mx, &my, &mz);
    }

    for (int i = 0; i < N; ++i) {
        float x = ax[i] - mx;
        float y = ay[i] - my;
        float z = az[i] - mz;

        // 单位换算：尽量只做一次
        if (st->cfg.unit == VIB_ACCEL_UNIT_G) {
            x *= g_to_ms2;
            y *= g_to_ms2;
            z *= g_to_ms2;
        }

        if (st->cfg.enable_bandpass) {
            x = vib_biquad_process(&st->hp10_x, x);
            y = vib_biquad_process(&st->hp10_y, y);
            z = vib_biquad_process(&st->hp10_z, z);

            x = vib_biquad_process(&st->lp_x, x);
            y = vib_biquad_process(&st->lp_y, y);
            z = vib_biquad_process(&st->lp_z, z);
        }

        // integrate to v (m/s)
        st->vx = st->vx * leak + x * dt;
        st->vy = st->vy * leak + y * dt;
        st->vz = st->vz * leak + z * dt;

        float vx_hp = vib_biquad_process(&st->hp1_vx, st->vx);
        float vy_hp = vib_biquad_process(&st->hp1_vy, st->vy);
        float vz_hp = vib_biquad_process(&st->hp1_vz, st->vz);

        vx_out[i] = vx_hp;
        vy_out[i] = vy_hp;
        vz_out[i] = vz_hp;
    }

    return VIB_ALGO_OK;
}
