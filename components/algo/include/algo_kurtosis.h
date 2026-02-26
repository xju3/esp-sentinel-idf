// features/vib_kurtosis.h
// 峭度（Kurtosis）特征：预留模块（便于后续扩展）
//
// 说明：
//  - 当前仅提供接口与最小实现骨架，避免把逻辑堆在 vibration_analyzer 中。

#ifndef VIB_KURTOSIS_H
#define VIB_KURTOSIS_H

#include "algo_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kx, ky, kz;
    float k3d;
} vib_kurtosis_out_t;

// 窗口计算峭度（未实现时返回 VIB_ALGO_UNSUPPORTED）
vib_algo_err_t vib_kurtosis_window(const float *x, const float *y, const float *z,
                                   int N, vib_kurtosis_out_t *out);

#ifdef __cplusplus
}
#endif

#endif // VIB_KURTOSIS_H
