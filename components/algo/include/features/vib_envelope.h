// features/vib_envelope.h
// 包络分析（Envelope Analysis）预留模块
//
// 典型用途：
//  - 轴承故障：带通（高频共振区）+ 绝对值/希尔伯特 + 低通得到包络，再做FFT。
//
// 当前仅提供接口骨架，后续可接入：
//  - Hilbert FIR / IIR 全通近似
//  - 绝对值近似 + LP

#ifndef VIB_ENVELOPE_H
#define VIB_ENVELOPE_H

#include "vib_algo_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // 预留：带通滤波器、解调滤波器状态等
    int reserved;
} vib_envelope_state_t;

typedef struct {
    // 预留：包络特征/峰值/频带能量等
    float reserved;
} vib_envelope_out_t;

void vib_envelope_init(vib_envelope_state_t *st);

vib_algo_err_t vib_envelope_window(vib_envelope_state_t *st,
                                  const float *x, const float *y, const float *z,
                                  int N, float fs_hz,
                                  vib_envelope_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // VIB_ENVELOPE_H
