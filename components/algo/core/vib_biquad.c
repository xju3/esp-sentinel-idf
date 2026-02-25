// core/vib_biquad.c

#include "core/vib_biquad.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool design_common(vib_biquad_df2t_t *q, float fc_hz, float fs_hz, bool is_lp)
{
    if (!q) return false;
    if (!(fs_hz > 0.0f)) return false;
    if (!(fc_hz > 0.0f)) return false;

    // 限制到 (0, fs/2)
    float nyq = 0.5f * fs_hz;
    if (fc_hz > 0.999f * nyq) fc_hz = 0.999f * nyq;

    const float Q = 0.7071067811865475f; // 1/sqrt(2)
    float w0 = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float cw = cosf(w0);
    float sw = sinf(w0);
    float alpha = sw / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;
    if (is_lp) {
        b0 = (1.0f - cw) * 0.5f;
        b1 = 1.0f - cw;
        b2 = (1.0f - cw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
    } else {
        b0 = (1.0f + cw) * 0.5f;
        b1 = -(1.0f + cw);
        b2 = (1.0f + cw) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
    }

    float inv_a0 = 1.0f / a0;
    q->b0 = b0 * inv_a0;
    q->b1 = b1 * inv_a0;
    q->b2 = b2 * inv_a0;
    q->a1 = a1 * inv_a0;
    q->a2 = a2 * inv_a0;
    // 不 reset 状态
    return true;
}

bool vib_biquad_design_butter2_lp(vib_biquad_df2t_t *q, float fc_hz, float fs_hz)
{
    return design_common(q, fc_hz, fs_hz, true);
}

bool vib_biquad_design_butter2_hp(vib_biquad_df2t_t *q, float fc_hz, float fs_hz)
{
    return design_common(q, fc_hz, fs_hz, false);
}
