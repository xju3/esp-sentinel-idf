#ifndef VIB_ALGO_ISO_10816_H
#define VIB_ALGO_ISO_10816_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef enum
{
    ISO_ZONE_A = 0,
    ISO_ZONE_B = 1,
    ISO_ZONE_C = 2,
    ISO_ZONE_D = 3
} iso_severity_zone_t;

iso_severity_zone_t get_iso_status(iso_machine_class_t machine_cls, float rms_velocity);
const char *get_zone_name(iso_severity_zone_t zone);


// Machine classes (ISO 10816-3 legacy thresholds)
typedef enum
{
    ISO_CLASS_I = 0,
    ISO_CLASS_II = 1,
    ISO_CLASS_III = 2,
    ISO_CLASS_IV = 3
} iso_machine_class_t;


typedef struct {
    // esp-dsp biquad 系数格式：b0,b1,b2,a1,a2 (假设 a0=1)
    // 这里用数组存放，便于直接调用 dsps_biquad_f32（ESP32-S3 有硬件优化实现）。
    float coef[5];

    // esp-dsp biquad 延迟线 w0,w1（Direct Form II）
    // 注意：该状态与 DF2T 的 s1/s2 定义不同，但两者实现的是同一差分方程。
    float w[2];
} algo_biquad_t;

typedef struct {
    float vx_rms;
    float vy_rms;
    float vz_rms;
    float v3d_rms;
} iso10816_result_t;

typedef struct {
    // Accel bandpass: 10 Hz HP + LP(fc_lp)
    algo_biquad_t hp10_x, hp10_y, hp10_z;
    algo_biquad_t lp_x,   lp_y,   lp_z;

    // Velocity high-pass: 1 Hz HP
    algo_biquad_t hp1_vx, hp1_vy, hp1_vz;

    // Velocity integrators (m/s)
    float vx, vy, vz;

    // Last fs used for coefficient design
    float fs;

    // Tunables
    float fc_lp_hz;      // default 1000 Hz (or min(1000, 0.45*fs))
    float skip_seconds;  // default 0.25s
    float fs_tol_hz;     // default 50 Hz
    float leak_hz;       // leaky integrator bleed freq (default 0.5 Hz)
} iso10816_state_t;

void iso10816_init(iso10816_state_t *st);

// Optional setters (can ignore if you want defaults)
void iso10816_set_fc_lp(iso10816_state_t *st, float fc_lp_hz);
void iso10816_set_skip_seconds(iso10816_state_t *st, float skip_seconds);
void iso10816_set_fs_tolerance(iso10816_state_t *st, float fs_tol_hz);
// void iso10816_set_leak_hz(iso10816_state_t *st, float leak_hz);

// Compute velocity RMS (mm/s RMS) for one window.
// ax/ay/az are acceleration in g.
// fs is sampling rate in Hz.
void iso10816_compute(iso10816_state_t *st,
                      const float *ax, const float *ay, const float *az,
                      int N, float fs, iso10816_result_t *out);
#ifdef __cplusplus
}
#endif

#endif // VIB_ALGO_ISO_10816_H
