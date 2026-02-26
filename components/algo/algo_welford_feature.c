// features/vib_welford_feature.c

#include "algo_welford_feature.h"
#include "algo_baseline.h"
#include <math.h>

static float one_axis_feature(const vib_welford_1d_t *s, float base_mean, float base_offset)
{
    // out = max(0, sqrt(var + (mean-base_mean)^2) - base_offset)
    float mean = vib_welford_1d_mean(s);
    float var = vib_welford_1d_var_sample(s);
    float dm = mean - base_mean;
    float raw = var + dm * dm;
    if (raw < 0.0f)
        raw = 0.0f;
    float rms = sqrtf(raw);
    float out = rms - base_offset;
    return (out > 0.0f) ? out : 0.0f;
}

vib_algo_err_t vib_welford_feature_from_stats(const vib_welford_3d_t *stats,
                                              vib_welford_feature_out_t *out)
{
    if (!stats || !out)
        return VIB_ALGO_BAD_ARGS;

    float bx = 0.0f, by = 0.0f, bz = 0.0f;
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;

    bx = g_baseline.x.val;
    by = g_baseline.y.val;
    bz = g_baseline.z.val;
    ox = g_baseline.x.offset;
    oy = g_baseline.y.offset;
    oz = g_baseline.z.offset;

    out->fx = one_axis_feature(&stats->x, bx, ox);
    out->fy = one_axis_feature(&stats->y, by, oy);
    out->fz = one_axis_feature(&stats->z, bz, oz);
    out->f3d = vib_3d_norm(out->fx, out->fy, out->fz);
    return VIB_ALGO_OK;
}
