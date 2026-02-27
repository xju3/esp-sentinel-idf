/*
 * algo_welford.c - Welford's Online Algorithm for Streaming Statistics
 *
 * O(1) space complexity algorithm for computing mean and variance incrementally.
 * Processes raw IMU data directly without intermediate float arrays.
 */

#include "algo_pdm.h"
#include "algo_welford.h"
#include "algo_dsp_utils.h"
#include <math.h>
#include <string.h>

// 初始化三轴 Welford 累计器
void algo_welford_init(vib_welford_t *ctx)
{
    if (ctx)
        memset(ctx, 0, sizeof(vib_welford_t));
}

// 在线更新三轴均值/方差（样本方差统计）
void algo_welford_update_1(vib_welford_t *ctx, float x, float y, float z)
{
    if (!ctx)
        return;
    ctx->count++;

    double dx = x - ctx->mean_x;
    ctx->mean_x += dx / ctx->count;
    ctx->m2_x += dx * (x - ctx->mean_x);

    double dy = y - ctx->mean_y;
    ctx->mean_y += dy / ctx->count;
    ctx->m2_y += dy * (y - ctx->mean_y);

    double dz = z - ctx->mean_z;
    ctx->mean_z += dz / ctx->count;
    ctx->m2_z += dz * (z - ctx->mean_z);
}

// 单轴基线校正后特征：max(0, sqrt(var+(mean-base)^2)-offset)
static float one_axis_feature(const vib_welford_1d_t *s, float base_mean, float base_offset)
{
    float mean = vib_welford_1d_mean(s);
    float var = vib_welford_1d_var_sample(s);
    float dm = mean - base_mean;
    float raw = var + dm * dm;
    float rms = sqrtf(raw < 0.0f ? 0.0f : raw);
    float out = rms - base_offset;
    return (out > 0.0f) ? out : 0.0f;
}

// 将已累积的三轴统计转成基线校正 RMS（fx/fy/fz/f3d）
vib_algo_err_t vib_welford_baseline_rms_from_stats(const vib_welford_3d_t *stats,
                                                   vib_welford_feature_out_t *out)
{
    if (!stats || !out)
        return VIB_ALGO_BAD_ARGS;

    float bx = g_baseline.x.val, by = g_baseline.y.val, bz = g_baseline.z.val;
    float ox = g_baseline.x.offset, oy = g_baseline.y.offset, oz = g_baseline.z.offset;

    out->fx = one_axis_feature(&stats->x, bx, ox);
    out->fy = one_axis_feature(&stats->y, by, oy);
    out->fz = one_axis_feature(&stats->z, bz, oz);
    out->f3d = vib_3d_norm(out->fx, out->fy, out->fz);
    return VIB_ALGO_OK;
}
