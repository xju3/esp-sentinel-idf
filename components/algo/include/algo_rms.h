#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "algo_pdm.h" // imu_raw_data_t, LSB_TO_G

#ifdef __cplusplus
extern "C" {
#endif

// 基础 RMS
float algo_calc_rms(const float *data, size_t count);

// ---------------- 速度 RMS 诊断（mm/s）----------------
// 说明：
//   - 面向 task_rms：输入原始 IMU chunk，内部完成：
//       去均值 -> 积分 -> 一阶高通 (fc≈1Hz) -> 累积平方和
//   - 无动态分配，状态驻留在调用方提供的结构体

typedef struct {
    // 采样步长/高通系数
    float dt;
    float hp_alpha;
    // 计数
    size_t count;
    // 积分状态 (m/s)
    float vx_ms, vy_ms, vz_ms;
    // 加速度运行均值用于去 DC
    float mean_ax, mean_ay, mean_az;
    // 一阶高通状态
    float hp_vx_prev_in,  hp_vy_prev_in,  hp_vz_prev_in;
    float hp_vx_prev_out, hp_vy_prev_out, hp_vz_prev_out;
    // 累积平方和 (mm/s)^2
    double sum_vx2, sum_vy2, sum_vz2;
} algo_rms_diag_state_t;

typedef struct {
    float rms_vx;
    float rms_vy;
    float rms_vz;
    float rms_v3d;
    size_t count;
    float fs_hz;
} algo_rms_diag_result_t;

void algo_rms_diag_init(algo_rms_diag_state_t *st, float dt, float hp_fc_hz);
void algo_rms_diag_update(algo_rms_diag_state_t *st,
                          const imu_raw_data_t *data, size_t count);
void algo_rms_diag_finish(const algo_rms_diag_state_t *st,
                          algo_rms_diag_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H
