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

#include "algo_err.h"
#include "algo_stash.h"
#include "algo_pdm.h"
#include "algo_baseline.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float fx, fy, fz;
    float f3d;
} vib_welford_feature_out_t;

// ---------------- Welford ----------------

typedef struct {
    uint32_t count;
    double m2_x, m2_y, m2_z;
    double mean_x, mean_y, mean_z;
} vib_welford_t;
;

void algo_welford_update_1(vib_welford_t *ctx, float x, float y, float z);

void algo_welford_init(vib_welford_t *ctx);

/**
 * @brief [WELFORD] 流式统计更新
 * @details 直接处理原始数据，无需中间float数组。
 *          极高的内存效率 - O(1)空间复杂度。
 * @param ctx Welford上下文（必须用零初始化）
 * @param src 原始IMU数据数组
 * @param count 要处理的样本数
 * @param axis 要处理的轴
 * @param sensitivity 灵敏度系数
 */
void algo_welford_update(algo_welford_ctx_t *ctx, 
                         const imu_raw_data_t *src, 
                         size_t count, 
                         algo_axis_t axis, 
                         float sensitivity);

// Baseline-corrected accel RMS feature (your existing logic):
// out = max(0, sqrt(var + (mean-base_mean)^2) - base_offset)
void algo_welford_finish(const vib_welford_t *ctx,
                         const vib_baseline_t *baseline,
                         float *out_x, float *out_y, float *out_z);

// 计算三轴 Welford baseline 校正特征
// 参数：
//  - stats: 已对窗口做过 update 的 Welford 状态
//  - baseline: 可为NULL；NULL时 base_mean=0 base_offset=0
//  - out: 输出
vib_algo_err_t vib_welford_feature_from_stats(const vib_welford_3d_t *stats,
                                             vib_welford_feature_out_t *out);


#ifdef __cplusplus
}
#endif

#endif // VIB_WELFORD_FEATURE_H
