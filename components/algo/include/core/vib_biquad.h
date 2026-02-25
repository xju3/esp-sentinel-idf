// core/vib_biquad.h
// DF2T biquad + RBJ cookbook 二阶 Butterworth LP/HP 设计
//
// 约束：
//  - 无动态内存
//  - 设计系数与 reset 状态分离：design 不会隐式清状态
//
// 使用场景：
//  - accel_to_velocity 中的 10Hz HP / LP / 1Hz HP
//  - 后续扩展：包络分析前的带通、解调滤波等

#ifndef VIB_BIQUAD_H
#define VIB_BIQUAD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // DF2T: y = b0*x + s1; s1 = b1*x - a1*y + s2; s2 = b2*x - a2*y
    float b0, b1, b2;
    float a1, a2; // a0=1 归一化后
    float s1, s2; // 状态
} vib_biquad_df2t_t;

// reset biquad state (s1/s2)
static inline void vib_biquad_reset(vib_biquad_df2t_t *q)
{
    if (!q) return;
    q->s1 = 0.0f;
    q->s2 = 0.0f;
}

// 处理单点采样
static inline float vib_biquad_process(vib_biquad_df2t_t *q, float x)
{
    // 注意：调用者需确保 q != NULL
    float y = q->b0 * x + q->s1;
    float s1 = q->b1 * x - q->a1 * y + q->s2;
    float s2 = q->b2 * x - q->a2 * y;
    q->s1 = s1;
    q->s2 = s2;
    return y;
}

// RBJ cookbook: 2阶 Butterworth (Q=1/sqrt(2))
// 参数：
//  - fc_hz: 截止频率（Hz）
//  - fs_hz: 采样率（Hz）
// 返回：true=成功，false=参数非法
bool vib_biquad_design_butter2_lp(vib_biquad_df2t_t *q, float fc_hz, float fs_hz);
bool vib_biquad_design_butter2_hp(vib_biquad_df2t_t *q, float fc_hz, float fs_hz);

#ifdef __cplusplus
}
#endif

#endif // VIB_BIQUAD_H
