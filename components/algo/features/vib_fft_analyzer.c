// features/vib_fft_analyzer.c

#include "features/vib_fft_analyzer.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float hann_coeff(int i, int N)
{
    // Hann: 0.5*(1-cos(2*pi*n/(N-1)))
    if (N <= 1) return 1.0f;
    return 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(N - 1)));
}

static void peaks_init(vib_fft_axis_out_t *out)
{
    if (!out || !out->peaks) return;
    for (uint16_t i = 0; i < out->npeaks; ++i) {
        out->peaks[i].freq_hz = 0.0f;
        out->peaks[i].amp = 0.0f;
        out->peaks[i].bin = 0;
    }
}

static void peaks_push(vib_fft_axis_out_t *out, uint16_t bin, float freq, float amp)
{
    // 简单插入排序：npeaks一般很小（3~10），O(n^2)可接受
    uint16_t n = out->npeaks;
    if (n == 0) return;

    // 若不够大，直接丢弃
    if (amp <= out->peaks[n - 1].amp) {
        return;
    }

    vib_fft_peak_t p = {.freq_hz = freq, .amp = amp, .bin = bin};
    int pos = (int)n - 1;
    out->peaks[pos] = p;
    while (pos > 0 && out->peaks[pos].amp > out->peaks[pos - 1].amp) {
        vib_fft_peak_t tmp = out->peaks[pos - 1];
        out->peaks[pos - 1] = out->peaks[pos];
        out->peaks[pos] = tmp;
        pos--;
    }
}

void vib_fft_analyzer_init(vib_fft_analyzer_t *an)
{
    if (!an) return;
    memset(an, 0, sizeof(*an));
    an->fs_tol_hz = 50.0f;
}

void vib_fft_analyzer_set_fs_tolerance(vib_fft_analyzer_t *an, float fs_tol_hz)
{
    if (!an) return;
    an->fs_tol_hz = fs_tol_hz;
}

vib_algo_err_t vib_fft_analyzer_set_cfg(vib_fft_analyzer_t *an, const vib_fft_analyzer_cfg_t *cfg)
{
    if (!an || !cfg) return VIB_ALGO_BAD_ARGS;
    if (cfg->N <= 0 || !(cfg->fs_hz > 0.0f)) return VIB_ALGO_BAD_ARGS;
    if (!vib_is_pow2_u32((uint32_t)cfg->N)) return VIB_ALGO_UNSUPPORTED_N;
    an->cfg = *cfg;
    return VIB_ALGO_OK;
}

static vib_algo_err_t ensure_twiddle(vib_fft_analyzer_t *an, vib_fft_workbuf_t *wb)
{
    if (!an || !wb) return VIB_ALGO_BAD_ARGS;
    int N = an->cfg.N;
    if (!wb->twiddle || wb->twiddle_len < 2 * N) return VIB_ALGO_NO_RESOURCE;

    // 当 N 改变 或 fs 改变超过 tol 时，重新生成 twiddle
    float tol = (an->fs_tol_hz > 0.0f) ? an->fs_tol_hz : 50.0f;
    bool need = (an->N_last != N);
    if (!need && an->fs_last > 0.0f && fabsf(an->fs_last - an->cfg.fs_hz) > tol) {
        // 这里 twiddle 与 fs 无关，但我们借用该机制保持与整体“fs抖动容忍”一致，
        // 同时避免上层误用导致 N/fs 配置频繁切换。
        need = false;
    }
    if (need) {
        vib_algo_err_t err = vib_fft_gen_twiddle_f32(wb->twiddle, N);
        if (err != VIB_ALGO_OK) return err;
        an->N_last = N;
        an->fs_last = an->cfg.fs_hz;
    }
    return VIB_ALGO_OK;
}

vib_algo_err_t vib_fft_analyze_axis(vib_fft_analyzer_t *an,
                                   const float *data,
                                   vib_fft_workbuf_t *wb,
                                   vib_fft_axis_out_t *out)
{
    if (!an || !data || !wb || !out) return VIB_ALGO_BAD_ARGS;
    int N = an->cfg.N;
    if (!out->peaks || out->npeaks != an->cfg.npeaks) return VIB_ALGO_BAD_ARGS;
    if (an->cfg.nbands > 0 && (!out->band_energy || out->nbands != an->cfg.nbands)) return VIB_ALGO_BAD_ARGS;
    if (!wb->work || wb->work_len < 2 * N) return VIB_ALGO_NO_RESOURCE;

    vib_algo_err_t err = ensure_twiddle(an, wb);
    if (err != VIB_ALGO_OK) return err;

    // 清输出
    peaks_init(out);
    if (out->band_energy && out->nbands) {
        for (uint16_t i = 0; i < out->nbands; ++i) out->band_energy[i] = 0.0f;
    }

    // 去均值
    double sum = 0.0;
    for (int i = 0; i < N; ++i) sum += data[i];
    float mean = (float)(sum / (double)N);

    // 填充 complex buffer
    for (int i = 0; i < N; ++i) {
        float v = data[i] - mean;
        if (an->cfg.use_hann) {
            v *= hann_coeff(i, N);
        }
        wb->work[2 * i + 0] = v;
        wb->work[2 * i + 1] = 0.0f;
    }

    err = vib_fft_cplx_radix2_inplace_f32(wb->work, N, wb->twiddle);
    if (err != VIB_ALGO_OK) return err;

    // 输出：仅看 [1 .. N/2-1]
    int half = N / 2;
    float df = an->cfg.fs_hz / (float)N;
    int min_bin = (int)ceilf(an->cfg.min_freq_hz / df);
    if (min_bin < 1) min_bin = 1;
    if (min_bin >= half) min_bin = half - 1;

    for (int k = min_bin; k < half; ++k) {
        float re = wb->work[2 * k + 0];
        float im = wb->work[2 * k + 1];
        float amp = sqrtf(re * re + im * im);
        float f = (float)k * df;

        // peaks
        peaks_push(out, (uint16_t)k, f, amp);

        // band energy: sum(amp^2) or sum(amp) 都可；这里用 amp^2 更接近能量
        if (an->cfg.bands && out->band_energy) {
            for (uint16_t b = 0; b < an->cfg.nbands; ++b) {
                float f0 = an->cfg.bands[b].f0_hz;
                float f1 = an->cfg.bands[b].f1_hz;
                if (f >= f0 && f < f1) {
                    out->band_energy[b] += amp * amp;
                }
            }
        }
    }

    return VIB_ALGO_OK;
}

vib_algo_err_t vib_fft_analyze_3axis(vib_fft_analyzer_t *an,
                                    const float *x, const float *y, const float *z,
                                    vib_fft_workbuf_t *wb,
                                    vib_fft_out_t *out)
{
    if (!an || !x || !y || !z || !wb || !out) return VIB_ALGO_BAD_ARGS;
    vib_algo_err_t err;
    err = vib_fft_analyze_axis(an, x, wb, &out->x);
    if (err != VIB_ALGO_OK) return err;
    err = vib_fft_analyze_axis(an, y, wb, &out->y);
    if (err != VIB_ALGO_OK) return err;
    err = vib_fft_analyze_axis(an, z, wb, &out->z);
    return err;
}
