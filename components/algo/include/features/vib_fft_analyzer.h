// features/vib_fft_analyzer.h
// FFT分析：去均值 + Hann窗 + real FFT(radix-2) + Top-N peaks + 频带能量
//
// 设计目标：
//  - 无动态内存：所有缓冲区由调用者提供
//  - 允许只输出压缩结果（peaks + bands）便于4G上报
//
// 说明：
//  - 对于 N 非2次幂：本实现选择“返回错误 VIB_ALGO_UNSUPPORTED_N”
//    （理由：工业监测常用固定点数，自动截断可能影响频率分辨率与统计一致性）。

#ifndef VIB_FFT_ANALYZER_H
#define VIB_FFT_ANALYZER_H

#include <stdint.h>
#include <stdbool.h>

#include "vib_algo_err.h"
#include "core/vib_fft.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float f0_hz;
    float f1_hz;
} vib_band_t;

typedef struct {
    float freq_hz;
    float amp; // 线性幅值（窗口后FFT幅值，未做严格物理归一化）
    uint16_t bin;
} vib_fft_peak_t;

typedef struct {
    // 输入约束
    int N;
    float fs_hz;

    // 行为
    float min_freq_hz;  // 默认 10Hz
    uint16_t npeaks;    // Top-N
    bool use_hann;      // 默认 true

    // 频带
    const vib_band_t *bands;
    uint16_t nbands;
} vib_fft_analyzer_cfg_t;

typedef struct {
    // 输出 peaks (长度 = cfg.npeaks)
    vib_fft_peak_t *peaks;
    uint16_t npeaks;

    // band energy (长度 = cfg.nbands)
    float *band_energy;
    uint16_t nbands;
} vib_fft_axis_out_t;

typedef struct {
    vib_fft_axis_out_t x;
    vib_fft_axis_out_t y;
    vib_fft_axis_out_t z;
} vib_fft_out_t;

// 工作内存：由调用者提供，避免malloc
typedef struct {
    // twiddle buffer: float[2*N]
    float *twiddle;
    int twiddle_len;

    // complex work buffer: float[2*N]
    float *work;
    int work_len;
} vib_fft_workbuf_t;

typedef struct {
    vib_fft_analyzer_cfg_t cfg;
    int N_last;
    float fs_last;
    float fs_tol_hz;
} vib_fft_analyzer_t;

// 初始化 analyzer（不会分配内存）
void vib_fft_analyzer_init(vib_fft_analyzer_t *an);

// 设置 fs 抖动容忍（默认50Hz）
void vib_fft_analyzer_set_fs_tolerance(vib_fft_analyzer_t *an, float fs_tol_hz);

// 配置（N/fs/npeaks/bands等）
vib_algo_err_t vib_fft_analyzer_set_cfg(vib_fft_analyzer_t *an, const vib_fft_analyzer_cfg_t *cfg);

// 对一个轴做FFT分析（输入时域 data[0..N-1]）
// out.peaks/out.band_energy 指针由调用者设置。
vib_algo_err_t vib_fft_analyze_axis(vib_fft_analyzer_t *an,
                                   const float *data,
                                   vib_fft_workbuf_t *wb,
                                   vib_fft_axis_out_t *out);

// 三轴分析
vib_algo_err_t vib_fft_analyze_3axis(vib_fft_analyzer_t *an,
                                    const float *x, const float *y, const float *z,
                                    vib_fft_workbuf_t *wb,
                                    vib_fft_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // VIB_FFT_ANALYZER_H
