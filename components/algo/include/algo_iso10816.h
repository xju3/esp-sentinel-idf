// features/vib_iso10816.h
// ISO10816/20816：速度RMS (mm/s RMS)
//
// 输入：三轴加速度窗口（g 或 m/s^2），采样率 fs
// 输出：vx/vy/vz/v3d 的速度RMS（mm/s）
//
// 说明：
//  - 速度由 pipeline/vib_accel_to_velocity 计算得到（m/s），再换算为 mm/s
//  - skip_seconds：在 pipeline 内部处理（used_from 返回跳过点）

#ifndef VIB_ISO10816_H
#define VIB_ISO10816_H

#include "algo_err.h"
#include "algo_accel_to_velocity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx_rms_mmps;
    float vy_rms_mmps;
    float vz_rms_mmps;
    float v3d_rms_mmps;
} vib_iso10816_out_t;

typedef struct {
    vib_accel_to_velocity_state_t a2v;
} vib_iso10816_state_t;

void vib_iso10816_init(vib_iso10816_state_t *st);

// 窗口计算：
//  - vx_buf/vy_buf/vz_buf: 调用者提供的临时缓冲区（长度>=N），避免库内部malloc
vib_algo_err_t vib_iso10816_compute_window(vib_iso10816_state_t *st,
                                          const float *ax, const float *ay, const float *az,
                                          int N, float fs_hz,
                                          float *vx_buf, float *vy_buf, float *vz_buf,
                                          vib_iso10816_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // VIB_ISO10816_H
