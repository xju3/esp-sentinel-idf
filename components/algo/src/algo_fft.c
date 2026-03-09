#include "algo_fft.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

// ESP-DSP 库头文件
#include "dsps_fft2r.h"
#include "dsps_math.h"
#include "dsps_wind_hann.h"

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

    // === 实数 FFT 优化 (N/2 Trick) ===
    // 将 N 个实数看作 N/2 个复数进行处理，内存需求减半，速度翻倍。
    // 申请暂存区: N * float (即 N/2 * Complex)
    float *y_cf = (float *)heap_caps_aligned_alloc(16, n * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!y_cf) {
        ESP_LOGE(TAG, "Failed to allocate FFT scratch buffer (%lu bytes)", n * sizeof(float));
        return ESP_ERR_NO_MEM;
    }

    // 2. 加窗 (Hanning Window)
    // 优化：使用 DSP 库生成窗函数和向量乘法，替代原本的慢速 cosf 循环
    // 步骤 A: 在 y_cf 中生成窗函数
    dsps_wind_hann_f32(y_cf, n);
    // 步骤 B: 应用窗函数 (y_cf = input * y_cf)
    // 注意：input 是 const，y_cf 既是窗也是输出 buffer
    dsps_mul_f32(input, y_cf, y_cf, n, 1, 1, 1);

    // 3. 执行 N/2 点复数 FFT
    // 此时 y_cf 中存储了 N 个实数，被视为 N/2 个复数 (Re, Im, Re, Im...)
    uint32_t n_complex = n / 2;

    // 位反转 (Bit Reverse)
    dsps_bit_rev_fc32(y_cf, n_complex);

    // FFT (基2, 复数)
    dsps_fft2r_fc32(y_cf, n_complex);

    // 4. 拆包 (Unpack) 与 幅值计算
    // 利用共轭对称性从 N/2 点复数 FFT 结果中恢复 N 点实数 FFT 结果
    // 归一化系数说明：
    // 1. FFT 基本缩放: 1/N
    // 2. 单边谱 (Single-Sided): * 2 (直流分量除外)
    // 3. Hanning 窗恢复系数 (Amplitude Correction Factor): * 2
    // 总系数 = (1/N) * 2 * 2 = 4/N
    float norm_factor = 4.0f / n;

    // 处理 DC (k=0) 和 Nyquist (k=N/2)
    // 对于 N/2 FFT 的第 0 点: Y[0] = Sum(x_even) + j*Sum(x_odd)
    // 真实的 X[0] = Re(Y[0]) + Im(Y[0])
    // 真实的 X[N/2] = Re(Y[0]) - Im(Y[0])
    float y0_r = y_cf[0];
    float y0_i = y_cf[1];
    
    // DC 分量 (0Hz)
    output[0] = fabsf(y0_r + y0_i) / n; // DC 不需要乘 2 和 2

    // 预计算旋转因子步长
    float angle_step = 2.0f * M_PI / n;

    // 循环计算 k = 1 到 N/2 - 1
    // 每次迭代同时计算 k 和 N/2 - k 两个点，但我们只需要前 N/2 个点的幅值
    // 由于 N/2 FFT 的输出是对称的，我们只需要遍历到 N/4
    for (uint32_t k = 1; k < n / 2; k++) {
        // 由于我们只需要幅值谱的前半部分 (0 ~ N/2)，
        // 这里需要通过 N/2 点 FFT 的结果 Y[k] 重构出 X[k]
        
        // 索引映射
        // Y[k]
        uint32_t idx_k = 2 * k;
        // Y[N/2 - k] (共轭对称点)
        uint32_t idx_nk = 2 * (n_complex - k);
        
        // 如果 k > N/4，其实是重复计算，但为了代码简单，我们只处理 k < N/2
        // 实际上 N/2 FFT 的结果只在 0 ~ N/4 有效，后半部分是共轭
        // 修正：我们需要遍历 k=1 到 N/4 (包含边界处理)
        // 但更简单的做法是直接利用公式计算 X[k]
        if (k > n / 4) break; // 优化：只需计算到 N/4，剩下的由对称性或不需要

        float yk_r = y_cf[idx_k];
        float yk_i = y_cf[idx_k + 1];
        float ynk_r = (k == 0) ? y0_r : y_cf[idx_nk];     // 边界保护，虽然 k 从 1 开始
        float ynk_i = (k == 0) ? y0_i : y_cf[idx_nk + 1];

        // 蝴蝶操作 (Butterfly) 恢复 X[k]
        float v_r = 0.5f * (yk_r + ynk_r);
        float v_i = 0.5f * (yk_i - ynk_i);
        float w_r = 0.5f * (yk_i + ynk_i);
        float w_i = -0.5f * (yk_r - ynk_r);

        // 旋转因子 W_N^k = exp(-j * 2*pi*k / N)
        float angle = -angle_step * k;
        float c = cosf(angle);
        float s = sinf(angle);

        // X[k] = (v_r + j*v_i) + (w_r + j*w_i) * (c + j*s)
        //      = (v_r + j*v_i) + ( (w_r*c - w_i*s) + j*(w_r*s + w_i*c) )
        float xk_r = v_r + (w_r * c - w_i * s);
        float xk_i = v_i + (w_r * s + w_i * c);
        
        output[k] = sqrtf(xk_r * xk_r + xk_i * xk_i) * norm_factor;

        // 同时可以算出 X[N/2 - k]
        if (k < n / 4) {
            float xnk_r = v_r - (w_r * c - w_i * s);
            float xnk_i = v_i - (w_r * s + w_i * c);
            output[n / 2 - k] = sqrtf(xnk_r * xnk_r + xnk_i * xnk_i) * norm_factor;
        }
        
        // 特殊处理 k = N/4 的情况 (此时 k = N/2 - k)
        // 上面的 if (k < n/4) 已经排除了重复赋值
    }

    // 释放暂存区
    heap_caps_free(y_cf);
    return ESP_OK;
}
