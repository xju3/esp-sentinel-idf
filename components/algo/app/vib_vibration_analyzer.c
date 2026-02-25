// app/vib_vibration_analyzer.c

#include "app/vib_vibration_analyzer.h"

#include <string.h>

static vib_analyzer_cfg_t default_cfg(void)
{
    vib_analyzer_cfg_t c;
    memset(&c, 0, sizeof(c));
    c.fs_fixed = 0.0f;
    c.fs_tol_hz = 50.0f;
    c.skip_seconds = 0.25f;
    c.welford_enable = true;
    c.iso_enable = true;
    c.fft_enable = false;
    c.fft_use_velocity = false;
    c.fft_npeaks = 5;
    c.fft_min_freq_hz = 10.0f;
    c.bands = NULL;
    c.nbands = 0;
    return c;
}

void vib_analyzer_init(vib_analyzer_state_t *st)
{
    if (!st) return;
    vib_iso10816_init(&st->iso_st);
    vib_fft_analyzer_init(&st->fft_an);
}

static void fill_fft_out(vib_analyzer_out_t *out,
                         vib_analyzer_fft_storage_t *s,
                         uint16_t npeaks,
                         const float *band_x, const float *band_y, const float *band_z,
                         uint16_t nbands)
{
    out->fft.x.peaks = s->peaks_x;
    out->fft.y.peaks = s->peaks_y;
    out->fft.z.peaks = s->peaks_z;
    out->fft.x.npeaks = npeaks;
    out->fft.y.npeaks = npeaks;
    out->fft.z.npeaks = npeaks;

    out->fft.x.band_energy = (float *)band_x;
    out->fft.y.band_energy = (float *)band_y;
    out->fft.z.band_energy = (float *)band_z;
    out->fft.x.nbands = nbands;
    out->fft.y.nbands = nbands;
    out->fft.z.nbands = nbands;
}

vib_algo_err_t vib_analyze_window(vib_analyzer_state_t *st,
                                 const float *ax, const float *ay, const float *az,
                                 int N, float fs_hz,
                                 const vib_baseline_t *baseline,
                                 const vib_analyzer_cfg_t *cfg_in,
                                 vib_analyzer_workbuf_t *wb,
                                 vib_analyzer_fft_storage_t *fft_store,
                                 vib_analyzer_out_t *out)
{
    if (!st || !ax || !ay || !az || !wb || !out) return VIB_ALGO_BAD_ARGS;
    if (N <= 0 || !(fs_hz > 0.0f)) return VIB_ALGO_BAD_ARGS;
    if (!vib_is_pow2_u32((uint32_t)N)) return VIB_ALGO_UNSUPPORTED_N;

    vib_analyzer_cfg_t cfg = cfg_in ? *cfg_in : default_cfg();
    float fs_use = (cfg.fs_fixed > 0.0f) ? cfg.fs_fixed : fs_hz;

    // 统一把 skip/fs_tol 下发到子模块
    st->iso_st.a2v.cfg.skip_seconds = cfg.skip_seconds;
    st->iso_st.a2v.cfg.fs_tol_hz = cfg.fs_tol_hz;
    vib_fft_analyzer_set_fs_tolerance(&st->fft_an, cfg.fs_tol_hz);

    memset(out, 0, sizeof(*out));

    // -------- Welford feature --------
    if (cfg.welford_enable) {
        vib_welford_3d_t w;
        vib_welford_3d_init(&w);
        for (int i = 0; i < N; ++i) {
            vib_welford_3d_update(&w, ax[i], ay[i], az[i]);
        }
        (void)vib_welford_feature_from_stats(&w, baseline, &out->welford);
    }

    // -------- ISO10816 velocity RMS --------
    if (cfg.iso_enable) {
        if (!wb->vx || !wb->vy || !wb->vz || wb->v_len < N) return VIB_ALGO_NO_RESOURCE;
        vib_algo_err_t err = vib_iso10816_compute_window(&st->iso_st,
                                                        ax, ay, az,
                                                        N, fs_use,
                                                        wb->vx, wb->vy, wb->vz,
                                                        &out->iso);
        if (err != VIB_ALGO_OK) return err;
    }

    // -------- FFT --------
    if (cfg.fft_enable) {
        if (!fft_store) return VIB_ALGO_BAD_ARGS;

        // bind output arrays
        fill_fft_out(out, fft_store, cfg.fft_npeaks,
                     fft_store->band_x, fft_store->band_y, fft_store->band_z, cfg.nbands);

        vib_fft_analyzer_cfg_t fcfg;
        memset(&fcfg, 0, sizeof(fcfg));
        fcfg.N = N;
        fcfg.fs_hz = fs_use;
        fcfg.min_freq_hz = (cfg.fft_min_freq_hz > 0.0f) ? cfg.fft_min_freq_hz : 10.0f;
        fcfg.npeaks = cfg.fft_npeaks;
        fcfg.use_hann = true;
        fcfg.bands = cfg.bands;
        fcfg.nbands = cfg.nbands;

        vib_algo_err_t err = vib_fft_analyzer_set_cfg(&st->fft_an, &fcfg);
        if (err != VIB_ALGO_OK) return err;

        if (!wb->fft_wb.twiddle || !wb->fft_wb.work) return VIB_ALGO_NO_RESOURCE;

        const float *fx = ax;
        const float *fy = ay;
        const float *fz = az;
        if (cfg.fft_use_velocity) {
            // 复用 ISO 的速度输出（m/s）做FFT
            if (!wb->vx || !wb->vy || !wb->vz || wb->v_len < N) return VIB_ALGO_NO_RESOURCE;
            fx = wb->vx;
            fy = wb->vy;
            fz = wb->vz;
        }

        err = vib_fft_analyze_3axis(&st->fft_an, fx, fy, fz, &wb->fft_wb, &out->fft);
        if (err != VIB_ALGO_OK) return err;
    }

    return VIB_ALGO_OK;
}
