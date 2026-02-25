// main/vib_demo.c
// 最小示例：展示如何初始化、设置baseline、调用 vib_analyze_window 输出结果
//
// 注意：该 demo 不会默认加入编译（避免影响现有固件）。
// 若要启用，请在 main/CMakeLists.txt 中把 vib_demo.c 加入 SRCS，或在此文件加宏控制。

#include "logger.h"

#include "app/vib_vibration_analyzer.h"

// 这里为了示例用静态数组；实际业务 N 典型 4096/8192/16384。
#define DEMO_N 4096

static float s_ax[DEMO_N];
static float s_ay[DEMO_N];
static float s_az[DEMO_N];

// 工作缓冲：速度(m/s) 与 FFT work/twiddle
static float s_vx[DEMO_N];
static float s_vy[DEMO_N];
static float s_vz[DEMO_N];
static float s_fft_twiddle[2 * DEMO_N];
static float s_fft_work[2 * DEMO_N];

// FFT 输出存储
static vib_fft_peak_t s_peaks_x[5];
static vib_fft_peak_t s_peaks_y[5];
static vib_fft_peak_t s_peaks_z[5];

// bands 示例：ISO 默认 10-1000Hz；FFT 可按需细分
static const vib_band_t s_bands[] = {
    {10.0f, 100.0f},
    {100.0f, 300.0f},
    {300.0f, 1000.0f},
};
static float s_band_x[sizeof(s_bands)/sizeof(s_bands[0])];
static float s_band_y[sizeof(s_bands)/sizeof(s_bands[0])];
static float s_band_z[sizeof(s_bands)/sizeof(s_bands[0])];

void vib_demo_run_once(void)
{
    // 1) 生成一些模拟数据（示例：50Hz 正弦）
    float fs = 4000.0f;
    for (int i = 0; i < DEMO_N; ++i) {
        float t = (float)i / fs;
        float s = sinf(2.0f * 3.1415926f * 50.0f * t);
        s_ax[i] = 0.02f * s;
        s_ay[i] = 0.01f * s;
        s_az[i] = 0.03f * s;
    }

    // 2) baseline
    vib_baseline_t bl;
    vib_baseline_zero(&bl);
    // 如需设置：
    // bl.x.val = ...; bl.x.offset = ...;

    // 3) cfg
    vib_analyzer_cfg_t cfg = {
        .fs_fixed = 0.0f,
        .fs_tol_hz = 50.0f,
        .skip_seconds = 0.25f,
        .welford_enable = true,
        .iso_enable = true,
        .fft_enable = true,
        .fft_use_velocity = false,
        .fft_npeaks = 5,
        .fft_min_freq_hz = 10.0f,
        .bands = s_bands,
        .nbands = (uint16_t)(sizeof(s_bands)/sizeof(s_bands[0])),
    };

    // 4) init state
    vib_analyzer_state_t st;
    vib_analyzer_init(&st);

    // 5) workbuf
    vib_analyzer_workbuf_t wb = {
        .vx = s_vx,
        .vy = s_vy,
        .vz = s_vz,
        .v_len = DEMO_N,
        .fft_wb = {
            .twiddle = s_fft_twiddle,
            .twiddle_len = (int)(2 * DEMO_N),
            .work = s_fft_work,
            .work_len = (int)(2 * DEMO_N),
        },
    };

    // 6) fft storage
    vib_analyzer_fft_storage_t fft_store = {
        .peaks_x = s_peaks_x,
        .peaks_y = s_peaks_y,
        .peaks_z = s_peaks_z,
        .band_x = s_band_x,
        .band_y = s_band_y,
        .band_z = s_band_z,
    };

    vib_analyzer_out_t out;
    vib_algo_err_t err = vib_analyze_window(&st,
                                            s_ax, s_ay, s_az,
                                            DEMO_N, fs,
                                            &bl,
                                            &cfg,
                                            &wb,
                                            &fft_store,
                                            &out);
    if (err != VIB_ALGO_OK) {
        LOG_ERRORF("vib_analyze_window failed: %d", (int)err);
        return;
    }

    LOG_INFOF("Welford feature (g RMS corrected): X=%.5f Y=%.5f Z=%.5f 3D=%.5f",
              out.welford.fx, out.welford.fy, out.welford.fz, out.welford.f3d);
    LOG_INFOF("ISO10816 velocity RMS (mm/s): X=%.3f Y=%.3f Z=%.3f 3D=%.3f",
              out.iso.vx_rms_mmps, out.iso.vy_rms_mmps, out.iso.vz_rms_mmps, out.iso.v3d_rms_mmps);

    LOG_INFOF("FFT peaks X: f=%.1fHz amp=%.3f ; Y: f=%.1fHz amp=%.3f ; Z: f=%.1fHz amp=%.3f",
              out.fft.x.peaks[0].freq_hz, out.fft.x.peaks[0].amp,
              out.fft.y.peaks[0].freq_hz, out.fft.y.peaks[0].amp,
              out.fft.z.peaks[0].freq_hz, out.fft.z.peaks[0].amp);
}
