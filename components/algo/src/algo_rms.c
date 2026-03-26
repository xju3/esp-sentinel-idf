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

static void dsps_rms_f32(const float *input, uint32_t len, float *result)
{
    float sum_sq = 0.0f;
    for (int i = 0; i < len; i++) {
        sum_sq += input[i] * input[i];
    }
    if (len > 0) {
        *result = sqrtf(sum_sq / len);
    } else {
        *result = 0.0f;
    }
}


static const char *TAG = "ALGO_RMS";

// 物理常数与滤波参数
#define G_TO_MM_S2 9806.65f   // 1g = 9806.65 mm/s^2
#define HPF_CUTOFF_HZ 10.0f   // 高通截止频率 (去除重力和漂移)
#define LPF_CUTOFF_HZ 1000.0f // 低通截止频率 (业务关注频段)
#define BIQUAD_Q 0.7071f      // Butterworth Q
#define STEADY_SKIP_MS 50U // 丢弃前 50ms 以避免滤波瞬态影响（可按需修改）

#define MAX_RMS_PROCESS_POINTS 8192
// 定义静态暂存区，强制 16 字节对齐以满足 SIMD 指令要求
EXT_RAM_BSS_ATTR static float s_scratch_buf[MAX_RMS_PROCESS_POINTS] __attribute__((aligned(16)));
// static SemaphoreHandle_t s_rms_mutex = NULL;

/**
 * @brief 处理单轴数据的内部辅助函数，计算完整的轴特征
 */
static axis_features_t process_axis(const float *input, uint32_t length, float sample_rate)
{
    axis_features_t result = {0};
    
    if (!input || length == 0)
        return result;
    if (length > MAX_RMS_PROCESS_POINTS)
    {
        ESP_LOGE(TAG, "Input length %lu exceeds static buffer size %d", length, MAX_RMS_PROCESS_POINTS);
        return result;
    }

    // 1. 使用静态暂存区 (无需申请)
    float *buf = s_scratch_buf;

    // 2. 去均值 + 单位转换与拷贝 (g -> mm/s^2)
    // 先去掉 DC (重力) 分量，避免每个窗口 HPF 产生较大瞬态
    float mean_g = 0.0f;
    for (int i = 0; i < length; i++)
    {
        mean_g += input[i];
    }
    mean_g /= (float)length;
    for (int i = 0; i < length; i++)
    {
        buf[i] = (input[i] - mean_g) * G_TO_MM_S2;
    }

    // 3. 第一级高通滤波 (HPF) - 去除重力分量 (DC)
    // 生成 2 阶 IIR Biquad 系数
    float coeffs_hpf[5];
    float w_hpf[2] = {0}; // 滤波器状态 (延迟线)
    // ESP-DSP expects normalized frequency (0..0.5) and Q factor
    float hpf_norm = HPF_CUTOFF_HZ / sample_rate;
    dsps_biquad_gen_hpf_f32(coeffs_hpf, hpf_norm, BIQUAD_Q);
    dsps_biquad_f32(buf, buf, length, coeffs_hpf, w_hpf);

    // 4. 低通滤波 (LPF) - 限制带宽到 1kHz
    float coeffs_lpf[5];
    float w_lpf[2] = {0};
    float lpf_norm = LPF_CUTOFF_HZ / sample_rate;
    dsps_biquad_gen_lpf_f32(coeffs_lpf, lpf_norm, BIQUAD_Q);
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

    // 7. 计算特征参数
    // 丢弃前一小段以避开滤波瞬态
    uint32_t skip = (uint32_t)(sample_rate * ((float)STEADY_SKIP_MS / 1000.0f));
    if (skip >= length)
        skip = 0;
    const float *feat_ptr = buf + skip;
    uint32_t feat_len = length - skip;
    if (feat_len == 0)
    {
        return result;
    }

    // 7.1 计算 RMS
    float rms = 0.0f;
    dsps_rms_f32(feat_ptr, feat_len, &rms);
    result.rms = rms;

    // 7.2 计算峰值 (Peak, 最大绝对值)
    float peak = 0.0f;
    float mav = 0.0f; // Mean Absolute Value (平均绝对值)
    for (uint32_t i = 0; i < feat_len; i++)
    {
        float abs_val = fabsf(feat_ptr[i]);
        if (abs_val > peak)
            peak = abs_val;
        mav += abs_val;
    }
    mav /= (float)feat_len;
    result.peak = peak;

    // 7.3 计算峰值因子 (Crest Factor = Peak / RMS)
    result.crest_factor = (rms > 0.0f) ? (peak / rms) : 0.0f;

    // 7.4 计算脉冲指标 (Impulse Factor = Peak / MAV)
    result.impulse_factor = (mav > 0.0f) ? (peak / mav) : 0.0f;

    return result;
}



vib_3axis_features_t algo_rms_calculate(const float *x, const float *y, const float *z, uint32_t length, float sample_rate)
{
    vib_3axis_features_t features = {0};

    // 依次处理三个轴，提取特征值
    // 这种串行处理方式可以最大程度节省堆内存（同一时间只占用一个轴的 scratch buffer）
    if (x)
        features.x_axis = process_axis(x, length, sample_rate);
    if (y)
        features.y_axis = process_axis(y, length, sample_rate);
    if (z)
        features.z_axis = process_axis(z, length, sample_rate);

    return features;
}
