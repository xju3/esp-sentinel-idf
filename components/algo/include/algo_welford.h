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
#include "algo_stats.h"
#include "algo_baseline.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration to avoid circular dependency with vib_baseline.h
// struct vib_baseline_s;
// typedef struct vib_baseline_s vib_baseline_t;

typedef struct {
    float fx, fy, fz;
    float f3d;
} vib_welford_feature_out_t;

// ---------------- Welford ----------------

typedef struct {
    uint32_t count;
    double m2_x, m2_y, m2_z;
    double mean_x, mean_y, mean_z;
} algo_welford_t;

typedef struct {
    uint32_t n;
    double mean;
    double m2;
} vib_welford_1d_t;

typedef struct {
    vib_welford_1d_t x;
    vib_welford_1d_t y;
    vib_welford_1d_t z;
} vib_welford_3d_t;




static inline void vib_welford_1d_init(vib_welford_1d_t *s)
{
    if (!s) return;
    s->n = 0;
    s->mean = 0.0;
    s->m2 = 0.0;
}

static inline void vib_welford_3d_init(vib_welford_3d_t *s)
{
    if (!s) return;
    vib_welford_1d_init(&s->x);
    vib_welford_1d_init(&s->y);
    vib_welford_1d_init(&s->z);
}

static inline void vib_welford_1d_update(vib_welford_1d_t *s, float v)
{
    if (!s) return;
    s->n++;
    double dv = (double)v - s->mean;
    s->mean += dv / (double)s->n;
    s->m2 += dv * ((double)v - s->mean);
}

static inline void vib_welford_3d_update(vib_welford_3d_t *s, float x, float y, float z)
{
    if (!s) return;
    vib_welford_1d_update(&s->x, x);
    vib_welford_1d_update(&s->y, y);
    vib_welford_1d_update(&s->z, z);
}

// 返回样本方差（n>1时 m2/(n-1)，否则 0）
static inline float vib_welford_1d_var_sample(const vib_welford_1d_t *s)
{
    if (!s || s->n <= 1) return 0.0f;
    return (float)(s->m2 / (double)(s->n - 1));
}

static inline float vib_welford_1d_mean(const vib_welford_1d_t *s)
{
    if (!s || s->n == 0) return 0.0f;
    return (float)s->mean;
}

static inline float vib_welford_1d_std_sample(const vib_welford_1d_t *s)
{
    float v = vib_welford_1d_var_sample(s);
    return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

// 3D 合成：sqrt(x^2+y^2+z^2)
static inline float vib_3d_norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void algo_welford_init(algo_welford_t *ctx);
void algo_welford_update(algo_welford_t *ctx, float x, float y, float z);

// Baseline-corrected accel RMS feature (your existing logic):
// out = max(0, sqrt(var + (mean-base_mean)^2) - base_offset)
void algo_welford_finish(const algo_welford_t *ctx,
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
