// features/vib_iso10816.c

#include "vib_iso10816.h"

#include <math.h>

static float rms_mmps_from_mps(const float *v_mps, int from, int N)
{
    if (!v_mps || N <= 0) return 0.0f;
    if (from < 0) from = 0;
    if (from >= N) return 0.0f;

    double sum2 = 0.0;
    int used = 0;
    for (int i = from; i < N; ++i) {
        float mmps = v_mps[i] * 1000.0f;
        sum2 += (double)mmps * (double)mmps;
        used++;
    }
    if (used <= 0) return 0.0f;
    return (float)sqrt(sum2 / (double)used);
}

void vib_iso10816_init(vib_iso10816_state_t *st)
{
    if (!st) return;
    vib_accel_to_velocity_init(&st->a2v);

    // ISO 默认 band: 10-1000Hz；velocity drift HP=1Hz。
    vib_accel_to_velocity_cfg_t cfg = st->a2v.cfg;
    cfg.unit = VIB_ACCEL_UNIT_G;
    cfg.enable_bandpass = true;
    cfg.enable_remove_mean = true;
    cfg.hp_accel_hz = 10.0f;
    cfg.lp_accel_hz = 1000.0f;
    cfg.hp_vel_hz = 1.0f;
    cfg.skip_seconds = 0.25f;
    cfg.fs_tol_hz = 50.0f;
    cfg.leak_hz = 0.5f;
    (void)vib_accel_to_velocity_set_cfg(&st->a2v, &cfg);
}

vib_algo_err_t vib_iso10816_compute_window(vib_iso10816_state_t *st,
                                          const float *ax, const float *ay, const float *az,
                                          int N, float fs_hz,
                                          float *vx_buf, float *vy_buf, float *vz_buf,
                                          vib_iso10816_out_t *out)
{
    if (!st || !ax || !ay || !az || !vx_buf || !vy_buf || !vz_buf || !out) return VIB_ALGO_BAD_ARGS;
    out->vx_rms_mmps = out->vy_rms_mmps = out->vz_rms_mmps = out->v3d_rms_mmps = 0.0f;

    int from = 0;
    vib_algo_err_t err = vib_accel_to_velocity_window(&st->a2v, ax, ay, az, N, fs_hz,
                                                     vx_buf, vy_buf, vz_buf, &from);
    if (err != VIB_ALGO_OK) return err;

    out->vx_rms_mmps = rms_mmps_from_mps(vx_buf, from, N);
    out->vy_rms_mmps = rms_mmps_from_mps(vy_buf, from, N);
    out->vz_rms_mmps = rms_mmps_from_mps(vz_buf, from, N);
    out->v3d_rms_mmps = sqrtf(out->vx_rms_mmps * out->vx_rms_mmps +
                              out->vy_rms_mmps * out->vy_rms_mmps +
                              out->vz_rms_mmps * out->vz_rms_mmps);
    return VIB_ALGO_OK;
}
