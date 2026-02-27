// core/vib_stats.h
// Welford 在线均值/方差 + RMS/mean/std + 3D合成工具
//
// 设计目标：
//  - 支持窗口模式（for循环 update）
//  - 未来可扩展：峭度、峰峰值、波形因子等

#ifndef VIB_STATS_H
#define VIB_STATS_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif // VIB_STATS_H
