#ifndef ALGO_FFT_H
#define ALGO_FFT_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 FFT 查找表
 * 在首次调用计算前会自动调用，也可以手动调用以预分配内存
 */
esp_err_t algo_fft_init(void);

/**
 * @brief 对单轴数据执行 FFT 并计算幅值谱 (Amplitude Spectrum)
 * 
 * @param input 输入时域信号 (实数)，长度必须为 n
 * @param output 输出频域幅值 (实数)，长度必须为 n/2
 * @param n FFT 点数 (必须是 2 的幂次方，当前最大支持 4096)
 * @return esp_err_t ESP_OK 成功，其他为错误码
 */
esp_err_t algo_fft_calculate(const float *input, float *output, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif // ALGO_FFT_H
