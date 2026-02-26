// pipeline/vib_accel_to_velocity.h
// 加速度 -> 速度（ISO10816/20816 速度RMS输入链路）
//
// 推荐流程：
//  1) (可选) 每窗口去均值（强制去DC，避免积分把偏置无限放大）
//  2) 加速度带通：默认 10Hz HP + LP(min(1000,0.45*fs))
//  3) 积分得到速度 v (m/s)
//  4) 速度再高通 1Hz 抑制漂移
//  5) skip_seconds：跳过 IIR 冷启动瞬态污染区
//
// 约束：
//  - 无动态内存
//  - 支持窗口 API；流式 API 提供状态结构，后续可扩展
//  - 支持 fs 抖动容忍：fs_tol_hz 内不重设系数、不reset状态

#ifndef VIB_ACCEL_TO_VELOCITY_H
#define VIB_ACCEL_TO_VELOCITY_H

#include <stdint.h>
#include <stdbool.h>

#include "algo_err.h"
#include "algo_biquad.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VIB_ACCEL_UNIT_G = 0,
    VIB_ACCEL_UNIT_MS2 = 1,
} vib_accel_unit_t;

typedef struct {
    // 配置
    vib_accel_unit_t unit;
    bool enable_bandpass;
    bool enable_remove_mean;

    float hp_accel_hz; // 默认 10
    float lp_accel_hz; // 默认 1000（最终 clamp 到 0.45*fs）
    float hp_vel_hz;   // 默认 1

    float skip_seconds; // 默认 0.25
    float fs_tol_hz;    // 默认 50
    float leak_hz;      // 积分器泄漏频率（默认 0.5）
} vib_accel_to_velocity_cfg_t;

typedef struct {
    // accel filters (per-axis)
    vib_biquad_df2t_t hp10_x, hp10_y, hp10_z;
    vib_biquad_df2t_t lp_x, lp_y, lp_z;

    // velocity HP (per-axis)
    vib_biquad_df2t_t hp1_vx, hp1_vy, hp1_vz;

    // integrators (m/s)
    float vx, vy, vz;

    // last-designed sample rate
    float fs_last;

    vib_accel_to_velocity_cfg_t cfg;
} vib_accel_to_velocity_state_t;

// 初始化 state + 默认配置
void vib_accel_to_velocity_init(vib_accel_to_velocity_state_t *st);

// 覆盖配置（不会隐式重置滤波器状态；如需强制，调用 vib_accel_to_velocity_reset_state）
vib_algo_err_t vib_accel_to_velocity_set_cfg(vib_accel_to_velocity_state_t *st,
                                            const vib_accel_to_velocity_cfg_t *cfg);

// reset 滤波器状态 + 积分器
void vib_accel_to_velocity_reset_state(vib_accel_to_velocity_state_t *st);

// 窗口处理：输入 g 或 m/s^2，输出速度（m/s）
// 说明：
//  - vx/vy/vz_out 指向调用者提供的缓冲区，长度至少 N
//  - 本函数会根据 fs_tol_hz 判断是否重设计系数；重设计时会 reset 状态
//  - used_from：输出有效起始点（skip 后的 index），用于后续 RMS/FFT 统计
vib_algo_err_t vib_accel_to_velocity_window(vib_accel_to_velocity_state_t *st,
                                           const float *ax, const float *ay, const float *az,
                                           int N, float fs_hz,
                                           float *vx_out, float *vy_out, float *vz_out,
                                           int *used_from);

#ifdef __cplusplus
}
#endif

#endif // VIB_ACCEL_TO_VELOCITY_H
