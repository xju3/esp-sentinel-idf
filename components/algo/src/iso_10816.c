#include "iso_10816.h"

void iso10816_init(iso10816_state_t *st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));

    // Defaults (you can override with setters)
    st->fc_lp_hz = 1000.0f;
    st->skip_seconds = 0.25f; // good for 1-second snapshots
    st->fs_tol_hz = 50.0f;
}

void iso10816_set_fc_lp(iso10816_state_t *st, float fc_lp_hz) {
    if (!st) return;
    st->fc_lp_hz = fc_lp_hz;
}

void iso10816_set_skip_seconds(iso10816_state_t *st, float skip_seconds) {
    if (!st) return;
    st->skip_seconds = skip_seconds;
}

void iso10816_set_fs_tolerance(iso10816_state_t *st, float fs_tol_hz) {
    if (!st) return;
    st->fs_tol_hz = fs_tol_hz;
}

void iso10816_compute(iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out)
{
    if (!st || !ax || !ay || !az || !out || N <= 0 || fs <= 0.0f) {
        return;
    }

    // Design (or reuse) coefficients
    iso10816_design_if_needed(st, fs);

    // Per-window independent measurement (your current workflow)
    st->vx = st->vy = st->vz = 0.0f;

    const float dt = 1.0f / fs;
    const float g_to_ms2 = 9.80665f;

    int skip = 0;
    if (st->skip_seconds > 0.0f) {
        skip = (int)lroundf(st->skip_seconds * fs);
        if (skip < 0) skip = 0;
        if (skip > N) skip = N;
    }

    double sum_vx2 = 0.0, sum_vy2 = 0.0, sum_vz2 = 0.0;
    int used = 0;

    for (int i = 0; i < N; ++i) {
        float x = ax[i] * g_to_ms2;
        float y = ay[i] * g_to_ms2;
        float z = az[i] * g_to_ms2;

        // Accel band-pass: 10Hz HP + LP
        x = algo_biquad_process(&st->hp10_x, x);
        y = algo_biquad_process(&st->hp10_y, y);
        z = algo_biquad_process(&st->hp10_z, z);

        x = algo_biquad_process(&st->lp_x, x);
        y = algo_biquad_process(&st->lp_y, y);
        z = algo_biquad_process(&st->lp_z, z);

        // Integrate to velocity (m/s)
        st->vx += x * dt;
        st->vy += y * dt;
        st->vz += z * dt;

        // Velocity HP at 1Hz
        float vx_hp = algo_biquad_process(&st->hp1_vx, st->vx);
        float vy_hp = algo_biquad_process(&st->hp1_vy, st->vy);
        float vz_hp = algo_biquad_process(&st->hp1_vz, st->vz);

        // Let filters settle before accumulating RMS
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

    out->vx_rms = (float)sqrt(sum_vx2 / used);
    out->vy_rms = (float)sqrt(sum_vy2 / used);
    out->vz_rms = (float)sqrt(sum_vz2 / used);

    out->v3d_rms = sqrtf(out->vx_rms * out->vx_rms +
                         out->vy_rms * out->vy_rms +
                         out->vz_rms * out->vz_rms);
}