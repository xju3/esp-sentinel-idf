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
EXT_RAM_BSS_ATTR static float s_band_buf[MAX_RMS_PROCESS_POINTS] __attribute__((aligned(16)));
// static SemaphoreHandle_t s_rms_mutex = NULL;

typedef struct
{
    float min_hz;
    float max_hz;
    float *out_rms;
} rms_band_spec_t;

static float normalized_cutoff(float cutoff_hz, float sample_rate)
{
    if (sample_rate <= 0.0f || cutoff_hz <= 0.0f)
    {
        return 0.0f;
    }

    float norm = cutoff_hz / sample_rate;
    if (norm < 0.0001f)
    {
        norm = 0.0001f;
    }
    if (norm > 0.49f)
    {
        norm = 0.49f;
    }
    return norm;
}

static void apply_hpf_inplace(float *buf, uint32_t length, float sample_rate, float cutoff_hz)
{
    if (!buf || length == 0U || cutoff_hz <= 0.0f)
    {
        return;
    }

    float coeffs[5];
    float state[2] = {0};
    dsps_biquad_gen_hpf_f32(coeffs, normalized_cutoff(cutoff_hz, sample_rate), BIQUAD_Q);
    dsps_biquad_f32(buf, buf, length, coeffs, state);
}

static void apply_lpf_inplace(float *buf, uint32_t length, float sample_rate, float cutoff_hz)
{
    if (!buf || length == 0U || cutoff_hz <= 0.0f)
    {
        return;
    }

    float coeffs[5];
    float state[2] = {0};
    dsps_biquad_gen_lpf_f32(coeffs, normalized_cutoff(cutoff_hz, sample_rate), BIQUAD_Q);
    dsps_biquad_f32(buf, buf, length, coeffs, state);
}

static esp_err_t prepare_velocity_trace(const float *input,
                                        uint32_t length,
                                        float sample_rate,
                                        float *buf,
                                        const float **out_feat_ptr,
                                        uint32_t *out_feat_len)
{
    if (!input || !buf || !out_feat_ptr || !out_feat_len || length == 0U || sample_rate <= 0.0f)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (length > MAX_RMS_PROCESS_POINTS)
    {
        ESP_LOGE(TAG, "Input length %lu exceeds static buffer size %d", length, MAX_RMS_PROCESS_POINTS);
        return ESP_ERR_INVALID_SIZE;
    }

    float mean_g = 0.0f;
    for (uint32_t i = 0; i < length; i++)
    {
        mean_g += input[i];
    }
    mean_g /= (float)length;
    for (uint32_t i = 0; i < length; i++)
    {
        buf[i] = (input[i] - mean_g) * G_TO_MM_S2;
    }

    apply_hpf_inplace(buf, length, sample_rate, HPF_CUTOFF_HZ);
    apply_lpf_inplace(buf, length, sample_rate, LPF_CUTOFF_HZ);

    float velocity = 0.0f;
    float prev_acc = 0.0f;
    const float dt = 1.0f / sample_rate;
    const float half_dt = 0.5f * dt;

    for (uint32_t i = 0; i < length; i++)
    {
        const float curr_acc = buf[i];
        velocity += (prev_acc + curr_acc) * half_dt;
        buf[i] = velocity;
        prev_acc = curr_acc;
    }

    apply_hpf_inplace(buf, length, sample_rate, HPF_CUTOFF_HZ);

    uint32_t skip = (uint32_t)(sample_rate * ((float)STEADY_SKIP_MS / 1000.0f));
    if (skip >= length)
    {
        skip = 0U;
    }

    *out_feat_ptr = buf + skip;
    *out_feat_len = length - skip;
    return (*out_feat_len > 0U) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static void fill_axis_debug(const float *input, uint32_t length, float sample_rate, axis_rms_debug_t *out_debug)
{
    if (!input || !out_debug)
    {
        return;
    }

    memset(out_debug, 0, sizeof(*out_debug));

    const float *feat_ptr = NULL;
    uint32_t feat_len = 0U;
    if (prepare_velocity_trace(input, length, sample_rate, s_scratch_buf, &feat_ptr, &feat_len) != ESP_OK)
    {
        return;
    }

    dsps_rms_f32(feat_ptr, feat_len, &out_debug->total_rms);

    const uint32_t half_len = feat_len / 2U;
    if (half_len > 0U)
    {
        dsps_rms_f32(feat_ptr, half_len, &out_debug->first_half_rms);
        dsps_rms_f32(feat_ptr + half_len, feat_len - half_len, &out_debug->second_half_rms);
    }

    rms_band_spec_t bands[] = {
        {10.0f, 30.0f, &out_debug->rms_10_30_hz},
        {30.0f, 80.0f, &out_debug->rms_30_80_hz},
        {80.0f, 200.0f, &out_debug->rms_80_200_hz},
        {200.0f, 500.0f, &out_debug->rms_200_500_hz},
        {500.0f, 1000.0f, &out_debug->rms_500_1000_hz},
    };

    for (uint32_t i = 0; i < (sizeof(bands) / sizeof(bands[0])); ++i)
    {
        memcpy(s_band_buf, feat_ptr, feat_len * sizeof(float));
        apply_hpf_inplace(s_band_buf, feat_len, sample_rate, bands[i].min_hz);
        apply_lpf_inplace(s_band_buf, feat_len, sample_rate, bands[i].max_hz);
        dsps_rms_f32(s_band_buf, feat_len, bands[i].out_rms);
    }
}

/**
 * @brief 处理单轴数据的内部辅助函数，计算完整的轴特征
 */
static axis_features_t process_axis(const float *input, uint32_t length, float sample_rate)
{
    axis_features_t result = {0};
    
    if (!input || length == 0U)
        return result;

    const float *feat_ptr = NULL;
    uint32_t feat_len = 0U;
    if (prepare_velocity_trace(input, length, sample_rate, s_scratch_buf, &feat_ptr, &feat_len) != ESP_OK)
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

esp_err_t algo_rms_calculate_debug(const float *x,
                                   const float *y,
                                   const float *z,
                                   uint32_t length,
                                   float sample_rate,
                                   vib_3axis_rms_debug_t *out_debug)
{
    if (!out_debug)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_debug, 0, sizeof(*out_debug));

    if (x)
    {
        fill_axis_debug(x, length, sample_rate, &out_debug->x_axis);
    }
    if (y)
    {
        fill_axis_debug(y, length, sample_rate, &out_debug->y_axis);
    }
    if (z)
    {
        fill_axis_debug(z, length, sample_rate, &out_debug->z_axis);
    }

    return ESP_OK;
}
