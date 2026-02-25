// core/vib_fft.c

#include "core/vib_fft.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vib_algo_err_t vib_fft_gen_twiddle_f32(float *tw, int N)
{
    if (!tw || N <= 0) return VIB_ALGO_BAD_ARGS;
    if (!vib_is_pow2_u32((uint32_t)N)) return VIB_ALGO_UNSUPPORTED_N;
    for (int k = 0; k < N; ++k) {
        float a = 2.0f * (float)M_PI * (float)k / (float)N;
        tw[2 * k + 0] = cosf(a);
        tw[2 * k + 1] = sinf(a);
    }
    return VIB_ALGO_OK;
}

// 反转 bits（最多 32bit）
static inline uint32_t bitrev_u32(uint32_t x, uint32_t bits)
{
    // 简单循环，N最大到16384(14bit)，成本可接受
    uint32_t r = 0;
    for (uint32_t i = 0; i < bits; ++i) {
        r = (r << 1) | (x & 1U);
        x >>= 1U;
    }
    return r;
}

vib_algo_err_t vib_fft_bitrev_inplace_f32(float *data, int N)
{
    if (!data || N <= 0) return VIB_ALGO_BAD_ARGS;
    if (!vib_is_pow2_u32((uint32_t)N)) return VIB_ALGO_UNSUPPORTED_N;

    // bits = log2(N)
    uint32_t bits = 0;
    for (uint32_t t = (uint32_t)N; t > 1U; t >>= 1U) bits++;

    for (uint32_t i = 0; i < (uint32_t)N; ++i) {
        uint32_t j = bitrev_u32(i, bits);
        if (j > i) {
            // swap complex
            float tr = data[2 * i + 0];
            float ti = data[2 * i + 1];
            data[2 * i + 0] = data[2 * j + 0];
            data[2 * i + 1] = data[2 * j + 1];
            data[2 * j + 0] = tr;
            data[2 * j + 1] = ti;
        }
    }
    return VIB_ALGO_OK;
}

vib_algo_err_t vib_fft_cplx_radix2_inplace_f32(float *data, int N, const float *tw)
{
    if (!data || !tw || N <= 0) return VIB_ALGO_BAD_ARGS;
    if (!vib_is_pow2_u32((uint32_t)N)) return VIB_ALGO_UNSUPPORTED_N;

    // Cooley-Tukey DIF/DIT：这里用 DIT（先bitrev后蝶形）更直观。
    vib_algo_err_t err = vib_fft_bitrev_inplace_f32(data, N);
    if (err != VIB_ALGO_OK) return err;

    for (int len = 2; len <= N; len <<= 1) {
        int half = len >> 1;
        int step = N / len; // twiddle step

        for (int i = 0; i < N; i += len) {
            for (int j = 0; j < half; ++j) {
                int idx0 = i + j;
                int idx1 = idx0 + half;

                // w = exp(-j*2pi*j/len) => 使用 tw 里正角度，取共轭 (cos, -sin)
                int tw_idx = j * step;
                float wr = tw[2 * tw_idx + 0];
                float wi = -tw[2 * tw_idx + 1];

                float r1 = data[2 * idx1 + 0];
                float i1 = data[2 * idx1 + 1];
                float tr = wr * r1 - wi * i1;
                float ti = wr * i1 + wi * r1;

                float r0 = data[2 * idx0 + 0];
                float i0 = data[2 * idx0 + 1];
                data[2 * idx0 + 0] = r0 + tr;
                data[2 * idx0 + 1] = i0 + ti;
                data[2 * idx1 + 0] = r0 - tr;
                data[2 * idx1 + 1] = i0 - ti;
            }
        }
    }

    return VIB_ALGO_OK;
}
