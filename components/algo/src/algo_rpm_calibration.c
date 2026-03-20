#include "algo_rpm_calibration.h"
#include "algo_fft.h"
#include "logger.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

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

    LOG_DEBUGF("Dominant axis: %d, P-P: %.3fg", best_axis, max_pp);
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

esp_err_t algo_rpm_calibration_calculate(
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
        LOG_ERROR("Failed to allocate memory for FFT magnitude");
        return ESP_ERR_NO_MEM;
    }

    // 2. 选择优势轴
    int best_axis = _find_dominant_axis(x_data, y_data, z_data, cfg.fft_size);
    const float *dominant_data[3] = {x_data, y_data, z_data};
    const float *signal = dominant_data[best_axis];

    // 3. 准备信号：去直流（需要可修改副本）
    float *sig_copy = (float *)heap_caps_malloc(cfg.fft_size * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!sig_copy) {
        LOG_ERROR("Failed to allocate memory for signal copy");
        heap_caps_free(fft_mag);
        return ESP_ERR_NO_MEM;
    }
    memcpy(sig_copy, signal, cfg.fft_size * sizeof(float));
    _remove_dc(sig_copy, cfg.fft_size);

    // 4. 执行 FFT
    ret = algo_fft_calculate(sig_copy, fft_mag, cfg.fft_size);
    if (ret != ESP_OK) {
        LOG_ERRORF("FFT calculation failed: %d", ret);
        goto cleanup;
    }

    // 5. 频谱转速计算
    ret = algo_rpm_calibration_from_spectrum(fft_mag, expected_rpm, &cfg, out_rpm);

cleanup:
    heap_caps_free(fft_mag);
    heap_caps_free(sig_copy);
    return ret;
}

esp_err_t algo_rpm_calibration_from_spectrum(
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
        LOG_INFOF("Calibration hint: %ld RPM. Searching window: %.1f - %.1f Hz", expected_rpm, low_freq, high_freq);
    } else {
        // 无参考转速：全范围盲搜
        min_bin = (int)(cfg.min_rpm / 60.0f / bin_res);
        max_bin = (int)(cfg.max_rpm / 60.0f / bin_res);
        LOG_INFO("Calibration blind mode (Full range search)");
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
        LOG_WARNF("Vibration too low for calibration (Max: %.4fg)", peak_mag);
        return ESP_FAIL;
    }

    // 转换频率到转速
    float freq = peak_idx * bin_res;
    *out_rpm = (int32_t)(freq * 60.0f);
    LOG_INFOF("Calibration Success: Peak=%.2f Hz, RPM=%ld, Mag=%.4fg", freq, *out_rpm, peak_mag);

    return ESP_OK;
}
