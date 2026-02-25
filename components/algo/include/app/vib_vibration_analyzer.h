// app/vib_vibration_analyzer.h
// 高层组合器：统一入口 vib_analyze_window()
//
// 输入：
//  - 三轴加速度窗口 ax/ay/az (g)，长度N
//  - 采样率 fs_hz（可选固定 fs_fixed）
//  - baseline（val/offset per-axis）
//
// 输出：
//  - Welford baseline 校正特征（每轴 + 3D）
//  - ISO10816/20816 速度RMS（mm/s，每轴 + 3D）
//  - FFT peaks + band energy（每轴）
//
// 约束：
//  - 无动态内存：所有工作缓冲/输出数组由调用者提供

#ifndef VIB_VIBRATION_ANALYZER_H
#define VIB_VIBRATION_ANALYZER_H

#include <stdint.h>
#include <stdbool.h>

#include "vib_algo_err.h"
#include "vib_baseline.h"

#include "core/vib_stats.h"

#include "features/vib_welford_feature.h"
#include "features/vib_iso10816.h"
#include "features/vib_fft_analyzer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 采样率
    float fs_fixed;      // =0 表示使用传入 fs_hz
    float fs_tol_hz;     // 默认 50

    // 冷启动瞬态跳过（ISO/速度链路等）
    float skip_seconds;  // 默认 0.25

    // enable
    bool welford_enable;
    bool iso_enable;
    bool fft_enable;

    // FFT
    bool fft_use_velocity;   // true: 用速度(m/s)做FFT；false: 用原始加速度(g)做FFT
    uint16_t fft_npeaks;
    float fft_min_freq_hz;   // 默认 10Hz

    // bands
    const vib_band_t *bands;
    uint16_t nbands;
} vib_analyzer_cfg_t;

typedef struct {
    // --- welford feature ---
    vib_welford_feature_out_t welford;

    // --- iso10816 ---
    vib_iso10816_out_t iso;

    // --- fft ---
    vib_fft_out_t fft;
} vib_analyzer_out_t;

// FFT 输出存储：调用者提供的数组
typedef struct {
    vib_fft_peak_t *peaks_x;
    vib_fft_peak_t *peaks_y;
    vib_fft_peak_t *peaks_z;

    float *band_x;
    float *band_y;
    float *band_z;
} vib_analyzer_fft_storage_t;

// 工作缓冲（无malloc）
typedef struct {
    // ISO 计算速度需要的临时数组（m/s）
    float *vx;
    float *vy;
    float *vz;
    int v_len;

    // FFT workbuf: twiddle + complex buffer
    vib_fft_workbuf_t fft_wb;
} vib_analyzer_workbuf_t;

typedef struct {
    vib_iso10816_state_t iso_st;
    vib_fft_analyzer_t fft_an;
} vib_analyzer_state_t;

// 初始化 analyzer state（内部会设置默认参数）
void vib_analyzer_init(vib_analyzer_state_t *st);

// 统一入口：窗口分析
// 参数：
//  - ax/ay/az: 加速度(g)
//  - N: 点数（必须 2^k，否则返回 VIB_ALGO_UNSUPPORTED_N）
//  - fs_hz: 当前采样率估计
//  - baseline: baseline校正参数，可为NULL
//  - cfg: 控制开关/FFT bands 等
//  - wb: 工作缓冲（速度数组 + FFT twiddle/work）
//  - fft_store: peaks/band 输出数组指针
//  - out: 输出
vib_algo_err_t vib_analyze_window(vib_analyzer_state_t *st,
                                 const float *ax, const float *ay, const float *az,
                                 int N, float fs_hz,
                                 const vib_baseline_t *baseline,
                                 const vib_analyzer_cfg_t *cfg,
                                 vib_analyzer_workbuf_t *wb,
                                 vib_analyzer_fft_storage_t *fft_store,
                                 vib_analyzer_out_t *out);

// 推荐默认 bands（注：这是建议值，不强制）。
//  - 4kHz: 10-100, 100-300, 300-1000
//  - 8kHz: 10-100, 100-300, 300-1000, 1000-2000, 2000-3000
//  - 16kHz: 10-100, 100-300, 300-1000, 1000-2000, 2000-4000, 4000-6000

#ifdef __cplusplus
}
#endif

#endif // VIB_VIBRATION_ANALYZER_H
