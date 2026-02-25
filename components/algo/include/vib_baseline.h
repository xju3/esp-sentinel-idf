// vib_baseline.h
// 通用 baseline 数据结构：val/offset per-axis
//
// 设计说明：
//  - 原工程已有 icm_freq_profile_t（val/offset per-axis）。为了让算法库更通用，
//    此处定义 vib_baseline_t。
//  - 上层 app 可以在必要时做一次类型转换/拷贝。

#ifndef VIB_BASELINE_H
#define VIB_BASELINE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float val;     // baseline mean（例如：静态零偏均值）
    float offset;  // baseline offset（例如：环境噪声RMS偏置）
} vib_axis_baseline_t;

typedef struct {
    vib_axis_baseline_t x;
    vib_axis_baseline_t y;
    vib_axis_baseline_t z;
} vib_baseline_t;

// 便捷：把 3轴 baseline val/offset 清零
static inline void vib_baseline_zero(vib_baseline_t *b)
{
    if (!b) return;
    b->x.val = b->y.val = b->z.val = 0.0f;
    b->x.offset = b->y.offset = b->z.offset = 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif // VIB_BASELINE_H
