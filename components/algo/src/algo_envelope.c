#include "algo_envelope.h"
#include "algo_fft.h"
#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"
#include "dsps_math.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ALGO_ENV";

#define MAX_SIG_LEN        8192
#define DEFAULT_ENV_BW     3000.0f // 带通带宽 3kHz
#define ENV_LPF_CUTOFF     500.0f  // 包络低通截止 500Hz
#define NOISE_FLOOR_EST    0.001f  // 简单的底噪估计 (g)

// === 静态缓冲区 (复用内存，避免堆碎片) ===
// 1. FFT 结果缓冲区 (复数/幅值)
EXT_RAM_BSS_ATTR static float s_env_fft_mag[MAX_SIG_LEN / 2] __attribute__((aligned(16)));

// 滤波器系数与状态
static float s_coeffs_bp_hpf[5];
static float s_coeffs_bp_lpf[5];
static float s_coeffs_env_lpf[5];
static float s_w_hpf[2];
static float s_w_lpf[2];
static float s_w_env[2];

static float s_last_fs = 0.0f;

esp_err_t algo_envelope_init(void)
{
    return ESP_OK;
}

static void update_filters(float fs)
{
    if (fabsf(fs - s_last_fs) < 1.0f) return;

    // 1. 带通滤波器 (2kHz - 5kHz)
    float f_start = 2000.0f;
    float f_end = f_start + DEFAULT_ENV_BW;
    if (f_end > fs / 2.0f) f_end = fs / 2.0f * 0.9f;

    dsps_biquad_gen_hpf_f32(s_coeffs_bp_hpf, f_start, fs);
    dsps_biquad_gen_lpf_f32(s_coeffs_bp_lpf, f_end, fs);

    // 2. 包络提取低通 (500Hz)
    dsps_biquad_gen_lpf_f32(s_coeffs_env_lpf, ENV_LPF_CUTOFF, fs);

    s_last_fs = fs;
    ESP_LOGD(TAG, "Filters updated for FS=%.1f", fs);
}

/**
 * @brief 在指定频率附近搜索峰值
 * @return 找到的峰值幅值，若未找到则返回 0.0
 */
static float find_peak_at_freq(const float *fft_mag, uint32_t fft_len, float bin_res, float target_freq, float *found_freq)
{
    if (target_freq <= 0.1f || target_freq >= (fft_len * bin_res)) return 0.0f;

    // 搜索窗口：±2% 或 ±2个Bin
    float tolerance = target_freq * 0.02f;
    if (tolerance < 2.0f * bin_res) tolerance = 2.0f * bin_res;

    int min_bin = (int)((target_freq - tolerance) / bin_res);
    int max_bin = (int)((target_freq + tolerance) / bin_res);

    if (min_bin < 1) min_bin = 1;
    if (max_bin >= fft_len) max_bin = fft_len - 1;

    float local_max = 0.0f;
    int local_idx = 0;

    for (int i = min_bin; i <= max_bin; i++) {
        if (fft_mag[i] > local_max) {
            local_max = fft_mag[i];
            local_idx = i;
        }
    }
    
    if (found_freq) {
        *found_freq = local_idx * bin_res;
    }
    return local_max;
}

/**
 * @brief 检查故障特征频率及其谐波
 */
static void check_fault_signature(const float *fft_mag, uint32_t fft_len, float bin_res, 
                                  float target_freq, fault_stat_t *out_stat)
{
    if (target_freq <= 0.1f) return;

    // 1. 检测基频 (1x)
    float freq_1x = 0.0f;
    float amp_1x = find_peak_at_freq(fft_mag, fft_len, bin_res, target_freq, &freq_1x);

    // 2. 检测二次谐波 (2x)
    float freq_2x = 0.0f;
    float amp_2x = find_peak_at_freq(fft_mag, fft_len, bin_res, target_freq * 2.0f, &freq_2x);

    // 3. 检测三次谐波 (3x)
    float freq_3x = 0.0f;
    float amp_3x = find_peak_at_freq(fft_mag, fft_len, bin_res, target_freq * 3.0f, &freq_3x);

    // 填充结果
    out_stat->amplitude = amp_1x;
    out_stat->frequency_hz = freq_1x; // 记录实际匹配到的基频

    // 诊断逻辑
    // 阈值设定：假设底噪约为 NOISE_FLOOR_EST，需显著高于底噪 (例如 3倍)
    float threshold = NOISE_FLOOR_EST * 3.0f;

    if (amp_1x > threshold) {
        out_stat->detected = true;
        // 基础置信度 60%
        out_stat->confidence = 0.6f;

        // 谐波加分
        if (amp_2x > threshold) {
            out_stat->confidence += 0.2f;
        }
        if (amp_3x > threshold) {
            out_stat->confidence += 0.1f;
        }
        
        // 幅值加分 (如果幅值非常大，置信度更高)
        if (amp_1x > threshold * 3.0f) {
            out_stat->confidence += 0.1f;
        }

        if (out_stat->confidence > 1.0f) out_stat->confidence = 1.0f;
    } else {
        out_stat->detected = false;
        out_stat->confidence = 0.0f;
    }
}

esp_err_t algo_envelope_execute(float *input_data, uint32_t len, float sample_rate, 
                                float rpm, const bearing_orders_t *bearing,
                                envelope_report_t *out_report)
{
    if (!input_data || !out_report || len == 0) return ESP_ERR_INVALID_ARG;
    if (len > MAX_SIG_LEN) len = MAX_SIG_LEN;

    update_filters(sample_rate);

    // DSP 流水线: BPF -> Abs -> LPF -> RemoveDC -> FFT
    memset(s_w_hpf, 0, sizeof(s_w_hpf));
    dsps_biquad_f32(input_data, input_data, len, s_coeffs_bp_hpf, s_w_hpf);
    memset(s_w_lpf, 0, sizeof(s_w_lpf));
    dsps_biquad_f32(input_data, input_data, len, s_coeffs_bp_lpf, s_w_lpf);
    for (uint32_t i = 0; i < len; i++) input_data[i] = fabsf(input_data[i]);
    memset(s_w_env, 0, sizeof(s_w_env));
    dsps_biquad_f32(input_data, input_data, len, s_coeffs_env_lpf, s_w_env);
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) mean += input_data[i];
    mean /= len;
    dsps_addc_f32(input_data, input_data, len, -mean, 1, 1);

    esp_err_t ret = algo_fft_calculate(input_data, s_env_fft_mag, len);
    if (ret != ESP_OK) return ret;

    float bin_res = sample_rate / (float)len;
    uint32_t search_limit_bin = (uint32_t)(ENV_LPF_CUTOFF / bin_res);
    if (search_limit_bin > len / 2) search_limit_bin = len / 2;

    out_report->peak_mag = 0.0f;
    out_report->peak_freq_hz = 0.0f;
    for (uint32_t i = 1; i < search_limit_bin; i++) {
        if (s_env_fft_mag[i] > out_report->peak_mag) {
            out_report->peak_mag = s_env_fft_mag[i];
            out_report->peak_freq_hz = i * bin_res;
        }
    }

    memset(&out_report->bpfo_stat, 0, sizeof(fault_stat_t));
    memset(&out_report->bpfi_stat, 0, sizeof(fault_stat_t));
    memset(&out_report->bs_stat, 0, sizeof(fault_stat_t));
    memset(&out_report->ftf_stat, 0, sizeof(fault_stat_t));

    if (bearing && rpm > 10.0f) {
        float freq_base = rpm / 60.0f;
        check_fault_signature(s_env_fft_mag, len / 2, bin_res, freq_base * bearing->bpfo, &out_report->bpfo_stat);
        check_fault_signature(s_env_fft_mag, len / 2, bin_res, freq_base * bearing->bpfi, &out_report->bpfi_stat);
        check_fault_signature(s_env_fft_mag, len / 2, bin_res, freq_base * bearing->bs,   &out_report->bs_stat);
        check_fault_signature(s_env_fft_mag, len / 2, bin_res, freq_base * bearing->ftf,  &out_report->ftf_stat);
    }

    return ESP_OK;
}