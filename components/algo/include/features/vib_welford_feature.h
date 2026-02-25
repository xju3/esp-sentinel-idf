// features/vib_welford_feature.h
// baseline-corrected accel RMS 特征（Welford统计）
//
// 输入：原始加速度 x/y/z（单位g）
// 输出：
//   out = max(0, sqrt(var + (mean-base_mean)^2) - base_offset)
//
// 说明：
//  - var 使用样本方差（n>1时 m2/(n-1)）
//  - base_mean/base_offset 来自 vib_baseline_t
//
// 使用场景：
//  - 对每个采样窗口做一次“基线校正后的加速度RMS”特征，用于趋势/报警。

#ifndef VIB_WELFORD_FEATURE_H
#define VIB_WELFORD_FEATURE_H

#include "vib_algo_err.h"
#include "vib_baseline.h"
#include "core/vib_stats.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float fx, fy, fz;
    float f3d;
} vib_welford_feature_out_t;

// 计算三轴 Welford baseline 校正特征
// 参数：
//  - stats: 已对窗口做过 update 的 Welford 状态
//  - baseline: 可为NULL；NULL时 base_mean=0 base_offset=0
//  - out: 输出
vib_algo_err_t vib_welford_feature_from_stats(const vib_welford_3d_t *stats,
                                             const vib_baseline_t *baseline,
                                             vib_welford_feature_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // VIB_WELFORD_FEATURE_H
