#include "algo_rms.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"

// ESP-DSP 库
#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"
#include "dsps_math.h"
#include "dsps_stats.h"

static const char *TAG = "ALGO_RMS";

// 物理常数与滤波参数
#define G_TO_MM_S2 9806.65f   // 1g = 9806.65 mm/s^2
#define HPF_CUTOFF_HZ 10.0f   // 高通截止频率 (去除重力和漂移)
#define LPF_CUTOFF_HZ 1000.0f // 低通截止频率 (业务关注频段)

#define MAX_RMS_PROCESS_POINTS 8192
// 定义静态暂存区，强制 16 字节对齐以满足 SIMD 指令要求
EXT_RAM_ATTR static float s_scratch_buf[MAX_RMS_PROCESS_POINTS] __attribute__((aligned(16)));
static SemaphoreHandle_t s_rms_mutex = NULL;

/**
 * @brief 处理单轴数据的内部辅助函数
 */
static float process_axis(const float *input, uint32_t length, float sample_rate)
{
    if (!input || length == 0)
        return 0.0f;
    if (length > MAX_RMS_PROCESS_POINTS)
    {
        ESP_LOGE(TAG, "Input length %lu exceeds static buffer size %d", length, MAX_RMS_PROCESS_POINTS);
        return 0.0f;
    }

    // 1. 使用静态暂存区 (无需申请)
    float *buf = s_scratch_buf;

    // 2. 单位转换与拷贝 (g -> mm/s^2)
    // dsps_mul_c_f32: output[i] = input[i * step_in] * C
    // 这里同时完成了数据从 const input 到 scratch buf 的拷贝
    dsps_mul_c_f32(input, buf, length, G_TO_MM_S2, 1, 1);

    // 3. 第一级高通滤波 (HPF) - 去除重力分量 (DC)
    // 生成 2 阶 IIR Biquad 系数
    float coeffs_hpf[5];
    float w_hpf[2] = {0}; // 滤波器状态 (延迟线)
    dsps_biquad_gen_hpf_f32(coeffs_hpf, HPF_CUTOFF_HZ, sample_rate);
    dsps_biquad_f32(buf, buf, length, coeffs_hpf, w_hpf);

    // 4. 低通滤波 (LPF) - 限制带宽到 1kHz
    float coeffs_lpf[5];
    float w_lpf[2] = {0};
    dsps_biquad_gen_lpf_f32(coeffs_lpf, LPF_CUTOFF_HZ, sample_rate);
    dsps_biquad_f32(buf, buf, length, coeffs_lpf, w_lpf);

    // 5. 时域积分 (Acceleration mm/s^2 -> Velocity mm/s)
    // 使用梯形积分公式: v[i] = v[i-1] + (a[i] + a[i-1]) * dt / 2
    float velocity = 0.0f;
    float prev_acc = 0.0f; // 假设初始加速度为 0 (经过 HPF 后合理)
    float dt = 1.0f / sample_rate;
    float half_dt = 0.5f * dt;

    for (int i = 0; i < length; i++)
    {
        float curr_acc = buf[i];
        // 积分累加
        velocity += (prev_acc + curr_acc) * half_dt;
        // 将 buffer 中的加速度替换为速度
        buf[i] = velocity;
        prev_acc = curr_acc;
    }

    // 6. 第二级高通滤波 (HPF) - 去除积分产生的趋势项/漂移
    // 复用之前的 HPF 系数，但必须重置状态 w_hpf
    memset(w_hpf, 0, sizeof(w_hpf));
    dsps_biquad_f32(buf, buf, length, coeffs_hpf, w_hpf);

    // 7. 计算 RMS
    float rms = 0.0f;
    dsps_rms_f32(buf, length, &rms);

    return rms;
}

vib_rms_t algo_rms_calculate(const float *x, const float *y, const float *z, uint32_t length, float sample_rate)
{
    vib_rms_t result = {0};

    // 依次处理三个轴
    // 这种串行处理方式可以最大程度节省堆内存（同一时间只占用一个轴的 scratch buffer）
    if (x)
        result.x = process_axis(x, length, sample_rate);
    if (y)
        result.y = process_axis(y, length, sample_rate);
    if (z)
        result.z = process_axis(z, length, sample_rate);

    return res
}
