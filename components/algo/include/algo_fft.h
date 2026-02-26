// core/vib_fft.h
// 简化版 radix-2 real FFT（无malloc）
//
// 设计说明：
//  - 为了满足“无动态内存malloc”的硬约束，同时又要保持可用的FFT性能，
//    我们采用：
//      - 调用者提供工作缓冲区（complex interleaved，长度 2*N float）
//      - 生成/缓存 twiddle 表（长度 2*N float）到用户提供的 buffer
//      - bit-reversal 使用通用 O(N logN) 交换（无查表、无malloc）
//  - 这里实现的是通用的 complex FFT + 由 real 输入转复数输入（imag=0）。
//
// 注意：
//  - 幅值标定：本库输出的是线性幅值（未做2/N等归一化的精确功率标定），
//    峰值/频带能量用于相对趋势/诊断足够；如需严格物理标定可再加校准系数。

#ifndef VIB_FFT_H
#define VIB_FFT_H

#include <stdint.h>
#include <stdbool.h>

#include "algo_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 判断是否为2次幂
static inline bool vib_is_pow2_u32(uint32_t n)
{
    return (n != 0) && ((n & (n - 1)) == 0);
}

// 生成 twiddle: w[k] = cos(2*pi*k/N) + j*sin(2*pi*k/N)
// 参数：
//  - tw: float[2*N] (interleaved cos,sin)
vib_algo_err_t vib_fft_gen_twiddle_f32(float *tw, int N);

// complex FFT in-place
// 参数：
//  - data: float[2*N] interleaved (re,im,...)
//  - tw: float[2*N] from vib_fft_gen_twiddle_f32
vib_algo_err_t vib_fft_cplx_radix2_inplace_f32(float *data, int N, const float *tw);

// bit reversal in-place (generic)
vib_algo_err_t vib_fft_bitrev_inplace_f32(float *data, int N);

#ifdef __cplusplus
}
#endif

#endif // VIB_FFT_H
