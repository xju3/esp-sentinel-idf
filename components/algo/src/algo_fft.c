#include "algo_fft.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

// ESP-DSP 库头文件
#include "dsps_fft2r.h"
#include "dsps_math.h"

static const char *TAG = "ALGO_FFT";
static bool s_fft_initialized = false;

// 假设最大支持 8192 点 FFT，这决定了查找表的大小
#define MAX_FFT_SIZE 8192

esp_err_t algo_fft_init(void)
{
    if (s_fft_initialized) {
        return ESP_OK;
    }
    
    // 初始化 FFT 查找表 (Bit-reverse table 和 Sin/Cos table)
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, MAX_FFT_SIZE);
    if (ret == ESP_OK) {
        s_fft_initialized = true;
        ESP_LOGI(TAG, "FFT tables initialized");
    } else {
        ESP_LOGE(TAG, "Failed to init FFT tables: %d", ret);
    }
    return ret;
}

esp_err_t algo_fft_calculate(const float *input, float *output, uint32_t n)
{
    if (!input || !output || n == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 检查 n 是否为 2 的幂次方
    if ((n & (n - 1)) != 0) {
        ESP_LOGE(TAG, "FFT size must be power of 2, got %lu", n);
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_fft_initialized) {
        esp_err_t err = algo_fft_init();
        if (err != ESP_OK) return err;
    }

    // 1. 申请复数暂存区 (2 * N * float)
    // 使用 aligned_alloc 确保内存对齐，利于 DSP 指令优化
    // 8192点 * 2 * 4字节 = 64KB，必须放在堆上 (推荐 SPIRAM，如果没有则用内部 SRAM)
    float *y_cf = (float *)heap_caps_aligned_alloc(16, 2 * n * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!y_cf) {
        ESP_LOGE(TAG, "Failed to allocate FFT scratch buffer (%lu bytes)", 2 * n * sizeof(float));
        return ESP_ERR_NO_MEM;
    }

    // 2. 加窗 (Hanning Window) 并填充复数数组实部
    // Hanning Window: w[i] = 0.5 * (1 - cos(2*PI*i / (N-1)))
    // 我们在拷贝数据的同时加窗，避免额外的内存开销
    float factor = 2.0f * M_PI / (float)(n - 1);
    for (uint32_t i = 0; i < n; i++) {
        float window = 0.5f * (1.0f - cosf(factor * i));
        y_cf[i * 2] = input[i] * window;     // Real part
        y_cf[i * 2 + 1] = 0.0f;              // Imaginary part
    }

    // 3. 位反转 (Bit Reverse) - esp-dsp FFT 需要此步骤
    dsps_bit_rev_fc32(y_cf, n);

    // 4. 执行 FFT (基2, 复数)
    dsps_fft2r_fc32(y_cf, n);

    // 5. 计算幅值谱并归一化
    // 归一化系数说明：
    // 1. FFT 基本缩放: 1/N
    // 2. 单边谱 (Single-Sided): * 2 (直流分量除外)
    // 3. Hanning 窗恢复系数 (Amplitude Correction Factor): * 2
    // 总系数 = (1/N) * 2 * 2 = 4/N
    float norm_factor = 4.0f / n;

    // 输出长度为 N/2 (只取前一半，忽略负频率)
    for (uint32_t i = 0; i < n / 2; i++) {
        float re = y_cf[i * 2];
        float im = y_cf[i * 2 + 1];
        float mag = sqrtf(re * re + im * im);
        
        if (i == 0) {
            // DC 分量 (0Hz) 不需要单边谱乘2，也不完全适用窗函数恢复，通常置0或单独处理
            // 这里简单处理为 1/N
            output[i] = mag / n; 
        } else {
            output[i] = mag * norm_factor;
        }
    }

    // 释放暂存区
    heap_caps_free(y_cf);
    return ESP_OK;
}