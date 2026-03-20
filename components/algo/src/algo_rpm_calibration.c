#include "algo_rpm_calibration.h"
#include "algo_fft.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// === 内部辅助函数 ===

/**
 * @brief 在三轴数据中选择优势轴 (Dominant Axis)
 * 
 * 策略：不做三轴 FFT 叠加（耗时），而是先在时域找出峰峰值最大的轴，
 * 只对该轴进行 FFT 分析
 * 
 * @param x_data X 轴数据
 * @param y_data Y 轴数据
 * @param z_data Z 轴数据
 * @param size 数据长度
 * @return int 优势轴索引 (0=X, 1=Y, 2=Z)
 */
static int _find_dominant_axis(const float *x_data, const float *y_data, const float *z_data, uint32_t size)
{
    const float *axes[3] = {x_data, y_data, z_data};
    int best_axis = 0;
    float max_pp = -1.0f;

    for (int axis = 0; axis < 3; axis++) {
        float min_v = 1000.0f;
        float max_v = -1000.0f;
        for (uint32_t i = 0; i < size; i++) {
            float v = axes[axis][i];
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
        float pp = max_v - min_v;
        if (pp > max_pp) {
            max_pp = pp;
            best_axis = axis;
        }
    }

    // LOG_DEBUGF("Dominant axis: %d, P-P: %.3fg", best_axis, max_pp);
    return best_axis;
}

/**
 * @brief 去直流分量 (Remove DC Component)
 * 
 * @param signal 输入/输出信号
 * @param size 数据长度
 */
static void _remove_dc(float *signal, uint32_t size)
{
    float mean = 0.0f;
    for (uint32_t i = 0; i < size; i++) {
        mean += signal[i];
    }
    mean /= size;

    for (uint32_t i = 0; i < size; i++) {
        signal[i] -= mean;
    }
}

/**
 * @brief 在 FFT 频谱中寻找转速峰值
 * 
 * @param fft_mag FFT 幅值谱
 * @param min_bin 搜索起始 bin
 * @param max_bin 搜索终止 bin
 * @param out_peak_bin 输出峰值 bin
 * @param out_peak_mag 输出峰值幅度
 */
static void _find_peak_in_range(
    const float *fft_mag,
    int min_bin,
    int max_bin,
    int *out_peak_bin,
    float *out_peak_mag)
{
    float max_val = 0.0f;
    int peak_idx = 0;

    for (int i = min_bin; i <= max_bin; i++) {
        if (fft_mag[i] > max_val) {
            max_val = fft_mag[i];
            peak_idx = i;
        }
    }

    *out_peak_bin = peak_idx;
    *out_peak_mag = max_val;
}

// === 公开接口实现 ===

rpm_calib_config_t algo_rpm_calib_get_default_config(void)
{
    return (rpm_calib_config_t){
        .odr_hz = 200.0f,
        .fft_size = 2048,
        .min_rpm = 600.0f,
        .max_rpm = 6000.0f,
        .min_mag_threshold_g = 0.005f,
    };
}

esp_err_t algo_rpm_calibration_auto_axis(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm)
{
    if (!x_data || !y_data || !z_data || !out_rpm) {
        return ESP_ERR_INVALID_ARG;
    }

    rpm_calib_config_t cfg = config ? *config : algo_rpm_calib_get_default_config();
    esp_err_t ret = ESP_OK;

    // 1. 内存分配：临时工作 buffer 用于 FFT
    float *fft_mag = (float *)heap_caps_malloc((cfg.fft_size / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!fft_mag) {
        // LOG_ERROR("Failed to allocate memory for FFT magnitude");
        return ESP_ERR_NO_MEM;
    }

    // 2. 选择优势轴
    int best_axis = _find_dominant_axis(x_data, y_data, z_data, cfg.fft_size);
    const float *dominant_data[3] = {x_data, y_data, z_data};
    const float *signal = dominant_data[best_axis];

    // 3. 准备信号：去直流（需要可修改副本）
    float *sig_copy = (float *)heap_caps_malloc(cfg.fft_size * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!sig_copy) {
        // LOG_ERROR("Failed to allocate memory for signal copy");
        heap_caps_free(fft_mag);
        return ESP_ERR_NO_MEM;
    }
    memcpy(sig_copy, signal, cfg.fft_size * sizeof(float));
    _remove_dc(sig_copy, cfg.fft_size);

    // 4. 执行 FFT
    ret = algo_fft_calculate(sig_copy, fft_mag, cfg.fft_size);
    if (ret != ESP_OK) {
        // LOG_ERRORF("FFT calculation failed: %d", ret);
        goto cleanup;
    }

    // 5. 频谱转速计算
    ret = algo_rpm_calibration_spectrum_search(fft_mag, expected_rpm, &cfg, out_rpm);

cleanup:
    heap_caps_free(fft_mag);
    heap_caps_free(sig_copy);
    return ret;
}

esp_err_t algo_rpm_calibration_spectrum_search(
    const float *fft_mag,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm)
{
    if (!fft_mag || !out_rpm) {
        return ESP_ERR_INVALID_ARG;
    }

    rpm_calib_config_t cfg = config ? *config : algo_rpm_calib_get_default_config();

    // 计算 FFT 频率分辨率
    float bin_res = cfg.odr_hz / (float)cfg.fft_size;
    int min_bin, max_bin;

    if (expected_rpm > 0) {
        // 有参考转速：开窗搜索 (±30%)
        // 目的：区分基频(1X)与谐波(2X, 3X)或皮带轮分频(0.5X)
        float center_freq = (float)expected_rpm / 60.0f;
        float low_freq = center_freq * 0.7f;
        float high_freq = center_freq * 1.3f;

        min_bin = (int)(low_freq / bin_res);
        max_bin = (int)(high_freq / bin_res);
        // LOG_INFOF("Calibration hint: %ld RPM. Searching window: %.1f - %.1f Hz", expected_rpm, low_freq, high_freq);
    } else {
        // 无参考转速：全范围盲搜
        min_bin = (int)(cfg.min_rpm / 60.0f / bin_res);
        max_bin = (int)(cfg.max_rpm / 60.0f / bin_res);
        // LOG_INFO("Calibration blind mode (Full range search)");
    }

    // 边界安全检查
    if (min_bin < 1) min_bin = 1; // 避开直流分量(Bin 0)
    if (max_bin >= (int)(cfg.fft_size / 2)) max_bin = cfg.fft_size / 2 - 1;

    // 寻峰
    int peak_idx = 0;
    float peak_mag = 0.0f;
    _find_peak_in_range(fft_mag, min_bin, max_bin, &peak_idx, &peak_mag);

    // 结果验证：检查幅值是否足够大
    if (peak_mag < cfg.min_mag_threshold_g) {
        // LOG_WARNF("Vibration too low for calibration (Max: %.4fg)", peak_mag);
        return ESP_FAIL;
    }

    // 转换频率到转速
    float freq = peak_idx * bin_res;
    *out_rpm = (int32_t)(freq * 60.0f);
    // LOG_INFOF("Calibration Success: Peak=%.2f Hz, RPM=%ld, Mag=%.4fg", freq, *out_rpm, peak_mag);

    return ESP_OK;
}

// ===================================================================
// 精细化转速标定 (使用谐波支持度和抛物线插值)
// ===================================================================

/**
 * @brief 应用 Hanning 窗口（海宁窗）
 * 
 * @param signal 输入信号（会被修改）
 * @param size 中断信号长度
 */
static void _apply_hanning_window(float *signal, uint32_t size)
{
    if (!signal || size < 2) return;

    for (uint32_t i = 0; i < size; i++) {
        float window_val = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)i / (float)(size - 1)));
        signal[i] *= window_val;
    }
}

/**
 * @brief 线性插值获取频谱值（用于评分时查询任意频率）
 * 
 * @param freqs 频率数组
 * @param spec FFT 幅值谱
 * @param spec_len 频谱长度
 * @param target_f 目标频率
 * @return 插值后的幅值
 */
static float _interp_amp_linear(
    const float *freqs,
    const float *spec,
    uint32_t spec_len,
    float target_f)
{
    if (!freqs || !spec || spec_len < 2) {
        return 0.0f;
    }

    // 找到最接近的 bin
    uint32_t best_idx = 0;
    float min_dist = fabsf(freqs[0] - target_f);
    
    for (uint32_t i = 1; i < spec_len; i++) {
        float dist = fabsf(freqs[i] - target_f);
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = i;
        }
    }

    // 如果目标频率超出范围，直接返回最接近的
    if (best_idx == 0 || best_idx >= spec_len - 1) {
        return spec[best_idx];
    }

    // 简单线性插值
    float f0 = freqs[best_idx - 1];
    float f1 = freqs[best_idx];
    float f2 = freqs[best_idx + 1];
    float s0 = spec[best_idx - 1];
    float s1 = spec[best_idx];
    float s2 = spec[best_idx + 1];

    if (target_f <= f1) {
        float alpha = (target_f - f0) / (f1 - f0 + 1e-10);
        return s0 + alpha * (s1 - s0);
    } else {
        float alpha = (target_f - f1) / (f2 - f1 + 1e-10);
        return s1 + alpha * (s2 - s1);
    }
}

/**
 * @brief 计算局部峰值的谐波支持度评分
 * 
 * 评分方式：f + 0.5*2f + 0.25*3f
 * （假设基频的谐波幅值逐次递减）
 * 
 * @param freqs 频率数组
 * @param spec FFT 幅值谱
 * @param spec_len 频谱长度
 * @param candidate_f 候选基频
 * @return 评分值
 */
static float _harmonic_support_score(
    const float *freqs,
    const float *spec,
    uint32_t spec_len,
    float candidate_f)
{
    float score = _interp_amp_linear(freqs, spec, spec_len, candidate_f);
    score += 0.5f * _interp_amp_linear(freqs, spec, spec_len, 2.0f * candidate_f);
    score += 0.25f * _interp_amp_linear(freqs, spec, spec_len, 3.0f * candidate_f);
    return score;
}

/**
 * @brief 三点抛物线插值精化峰值位置
 * 
 * 给定 FFT 结果中相邻三个点的幅值，使用抛物线拟合找出更精确的最大值位置
 * 
 * @param y1 左邻近点幅值
 * @param y2 中心点幅值
 * @param y3 右邻近点幅值
 * @param bin_width 相邻 bin 之间的频率宽度
 * @param center_freq 中心点对应的频率
 * @return 精化后的频率
 */
static float _parabolic_interpolation(
    float y1, float y2, float y3,
    float bin_width, float center_freq)
{
    float denom = (y1 - 2.0f * y2 + y3);
    
    if (fabsf(denom) < 1e-12f) {
        // 近似为线性，直接返回中心
        return center_freq;
    }

    float delta = 0.5f * (y1 - y3) / denom;
    
    // 限制偏移，防止过度插值
    if (fabsf(delta) > 0.5f) {
        delta = (delta > 0.0f) ? 0.5f : -0.5f;
    }

    return center_freq + delta * bin_width;
}

esp_err_t algo_rpm_calibration_harmonic_score(
    const float *signal,
    uint32_t signal_length,
    float sampling_rate,
    int32_t sync_rpm,
    int32_t max_slip_rpm,
    uint32_t pad_factor,
    rpm_harmonic_result_t *out_result)
{
    if (!signal || signal_length < 16 || !out_result || sampling_rate <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pad_factor < 1) {
        pad_factor = 1;
    }

    // 计算 FFT 点数：原始长度的整倍数
    uint32_t n_fft = signal_length * pad_factor;

    // 内存分配
    float *sig_windowed = (float *)heap_caps_malloc(signal_length * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!sig_windowed) {
        return ESP_ERR_NO_MEM;
    }

    float *spec = (float *)heap_caps_malloc((n_fft / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!spec) {
        heap_caps_free(sig_windowed);
        return ESP_ERR_NO_MEM;
    }

    float *freqs = (float *)heap_caps_malloc((n_fft / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!freqs) {
        heap_caps_free(sig_windowed);
        heap_caps_free(spec);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;

    // 1. 去直流并应用汉宁窗
    memcpy(sig_windowed, signal, signal_length * sizeof(float));
    _remove_dc(sig_windowed, signal_length);
    _apply_hanning_window(sig_windowed, signal_length);

    // 2. 执行 FFT（使用零填充）
    // 注意：algo_fft_calculate 需要处理信号长度 <= n_fft，这里需要额外的 FFT 实现或调整
    // 临时方案：使用 signal_length 的 FFT，然后不使用零填充优势
    // 更好方案：直接使用 esp_dsp 的 FFT 或其他支持零填充的库
    
    // 为了兼容当前的 algo_fft_calculate（只处理原始长度），我们先做不考虑零填充的版本
    // 如果需要零填充，需要自己实现或更换 FFT 库
    
    uint32_t fft_size = signal_length;
    uint32_t spec_bins = fft_size / 2;

    ret = algo_fft_calculate(sig_windowed, spec, fft_size);
    if (ret != ESP_OK) {
        goto cleanup;
    }

    // 3. 计算频率数组
    float bin_hz = sampling_rate / (float)fft_size;
    for (uint32_t i = 0; i < spec_bins; i++) {
        freqs[i] = i * bin_hz;
    }

    // 4. 确定搜索频带
    float f_sync = (float)sync_rpm / 60.0f;
    float f_min = (float)(sync_rpm - max_slip_rpm) / 60.0f;
    float f_max = f_sync - 0.02f;

    if (f_min <= 0.0f) f_min = 0.1f;
    if (f_max <= f_min) f_max = f_sync;

    // 5. 在搜索频带内寻峰值，使用谐波支持度评分
    uint32_t best_peak_idx = 0;
    float best_score = 0.0f;

    for (uint32_t i = 1; i < spec_bins - 1; i++) {
        float freq_i = freqs[i];

        // 只在搜索频带内评估
        if (freq_i < f_min || freq_i > f_max) {
            continue;
        }

        // 检查是否为局部最大值
        if (spec[i] <= spec[i - 1] || spec[i] <= spec[i + 1]) {
            continue;
        }

        // 计算谐波支持度
        float score = _harmonic_support_score(freqs, spec, spec_bins, freq_i);

        if (score > best_score) {
            best_score = score;
            best_peak_idx = i;
        }
    }

    if (best_peak_idx == 0 || best_score <= 0.0f) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 6. 三点抛物线插值精化峰值位置
    float refined_freq = _parabolic_interpolation(
        spec[best_peak_idx - 1],
        spec[best_peak_idx],
        spec[best_peak_idx + 1],
        bin_hz,
        freqs[best_peak_idx]
    );

    // 7. 计算置信度：峰值幅度 / 噪声底
    float peak_amp = _interp_amp_linear(freqs, spec, spec_bins, refined_freq);
    float noise_floor = 0.0f;
    
    // 估计噪声底为谱的中位数
    for (uint32_t i = 0; i < spec_bins; i++) {
        noise_floor += spec[i];
    }
    noise_floor = noise_floor / (float)spec_bins;
    if (noise_floor < 1e-12f) noise_floor = 1e-12f;

    float confidence = peak_amp / noise_floor;

    // 8. 填充输出结果
    out_result->freq_hz = refined_freq;
    out_result->rpm = refined_freq * 60.0f;
    out_result->confidence = confidence;
    out_result->window_seconds = (float)signal_length / sampling_rate;
    out_result->rayleigh_hz = sampling_rate / (float)fft_size;

cleanup:
    heap_caps_free(sig_windowed);
    heap_caps_free(spec);
    heap_caps_free(freqs);

    return ret;
}

// ===================================================================
// 第4种方法：基于物理滑差频带的 RPM 标定
// ===================================================================

/**
 * @brief 在给定频率范围内找所有真实波峰（局部最大值）
 * 
 * @param spec FFT幅值谱
 * @param spec_len 频谱长度
 * @param min_idx 搜索起始索引
 * @param max_idx 搜索终止索引
 * @param height_threshold 幅值阈值（低于此值的峰被过滤）
 * @param peak_indices 输出：峰值索引数组（预分配，最多capacity个）
 * @param peak_count 输出：实际找到的峰值数量
 * @param capacity 峰值数组容纳上限
 */
static void _find_all_peaks_in_range(
    const float *spec,
    uint32_t spec_len,
    uint32_t min_idx,
    uint32_t max_idx,
    float height_threshold,
    uint32_t *peak_indices,
    uint32_t *peak_count,
    uint32_t capacity)
{
    if (!spec || !peak_indices || !peak_count || capacity == 0) {
        if (peak_count) *peak_count = 0;
        return;
    }

    uint32_t count = 0;

    // 遍历搜索范围内的所有点，检查是否为局部最大值
    for (uint32_t i = min_idx + 1; i < max_idx; i++) {
        // 局部最大值条件：当前点 > 左邻点 && 当前点 > 右邻点
        if (spec[i] > spec[i - 1] && spec[i] > spec[i + 1]) {
            // 幅值阈值过滤
            if (spec[i] >= height_threshold) {
                if (count < capacity) {
                    peak_indices[count] = i;
                    count++;
                }
            }
        }
    }

    *peak_count = count;
}

esp_err_t algo_rpm_calibration_slip_model(
    const float *signal,
    uint32_t signal_length,
    float sampling_rate,
    int32_t sync_rpm,
    int32_t max_slip_rpm,
    uint32_t pad_factor,
    rpm_slip_model_result_t *out_result)
{
    if (!signal || signal_length < 16 || !out_result || sampling_rate <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sync_rpm <= 0 || max_slip_rpm < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (pad_factor < 1) {
        pad_factor = 1;
    }

    // 内存分配
    float *sig_windowed = (float *)heap_caps_malloc(signal_length * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!sig_windowed) {
        return ESP_ERR_NO_MEM;
    }

    float *spec = (float *)heap_caps_malloc((signal_length / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!spec) {
        heap_caps_free(sig_windowed);
        return ESP_ERR_NO_MEM;
    }

    float *freqs = (float *)heap_caps_malloc((signal_length / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!freqs) {
        heap_caps_free(sig_windowed);
        heap_caps_free(spec);
        return ESP_ERR_NO_MEM;
    }

    uint32_t max_peaks = 32;
    uint32_t *peak_indices = (uint32_t *)heap_caps_malloc(max_peaks * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (!peak_indices) {
        heap_caps_free(sig_windowed);
        heap_caps_free(spec);
        heap_caps_free(freqs);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;

    // 1. 去直流并应用Hanning窗
    memcpy(sig_windowed, signal, signal_length * sizeof(float));
    _remove_dc(sig_windowed, signal_length);
    _apply_hanning_window(sig_windowed, signal_length);

    // 2. 执行FFT（注：算法中提到5倍零填充，这里用实际采样点数替代）
    // 如需真正零填充，需使用支持变长FFT的库或自行实现
    uint32_t fft_size = signal_length;
    uint32_t spec_bins = fft_size / 2;

    ret = algo_fft_calculate(sig_windowed, spec, fft_size);
    if (ret != ESP_OK) {
        goto cleanup;
    }

    // 3. 计算频率数组
    float bin_hz = sampling_rate / (float)fft_size;
    for (uint32_t i = 0; i < spec_bins; i++) {
        freqs[i] = i * bin_hz;
    }

    // 4. 物理滑差频带：[sync_rpm - max_slip_rpm, sync_rpm - 0.04Hz]
    float freq_sync = (float)sync_rpm / 60.0f;
    float freq_min = (float)(sync_rpm - max_slip_rpm) / 60.0f;
    float freq_max = freq_sync - 0.04f;  // 刻意避开同步频率

    if (freq_min <= 0.0f) freq_min = 0.1f;
    if (freq_max <= freq_min) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 转换为索引范围
    uint32_t min_idx = (uint32_t)(freq_min / bin_hz);
    uint32_t max_idx = (uint32_t)(freq_max / bin_hz);

    if (min_idx < 1) min_idx = 1;
    if (max_idx >= spec_bins) max_idx = spec_bins - 1;

    // 5. 寻找频带内所有波峰（必须是局部最大值）
    // 阈值设为频段最大值的 5%
    float seg_max = 0.0f;
    for (uint32_t i = min_idx; i <= max_idx; i++) {
        if (spec[i] > seg_max) {
            seg_max = spec[i];
        }
    }

    float height_threshold = seg_max * 0.05f;
    if (height_threshold < 1e-12f) {
        height_threshold = 1e-12f;
    }

    uint32_t peak_count = 0;
    _find_all_peaks_in_range(spec, spec_bins, min_idx, max_idx, height_threshold, 
                             peak_indices, &peak_count, max_peaks);

    if (peak_count == 0) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    // 6. 在所有波峰中选幅度最大的
    uint32_t best_peak_idx = peak_indices[0];
    float best_peak_amp = spec[best_peak_idx];

    for (uint32_t i = 1; i < peak_count; i++) {
        uint32_t idx = peak_indices[i];
        if (spec[idx] > best_peak_amp) {
            best_peak_amp = spec[idx];
            best_peak_idx = idx;
        }
    }

    // 7. 填充输出结果
    float best_freq = freqs[best_peak_idx];
    out_result->freq_hz = best_freq;
    out_result->rpm = best_freq * 60.0f;
    out_result->peak_amplitude = best_peak_amp;
    out_result->window_seconds = (float)signal_length / sampling_rate;
    out_result->rayleigh_hz = sampling_rate / (float)fft_size;

cleanup:
    heap_caps_free(sig_windowed);
    heap_caps_free(spec);
    heap_caps_free(freqs);
    heap_caps_free(peak_indices);

    return ret;
}
