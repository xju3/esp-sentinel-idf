#ifndef ALGO_RPM_CALIBRATION_H
#define ALGO_RPM_CALIBRATION_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RPM 标定配置参数
 */
typedef struct {
    float odr_hz;               ///< 采样率 (Hz)，默认 200Hz
    uint32_t fft_size;          ///< FFT 点数，默认 2048
    float min_rpm;              ///< 最小关注转速 (RPM)，默认 600
    float max_rpm;              ///< 最大关注转速 (RPM)，默认 6000
    float min_mag_threshold_g;  ///< 最小幅值阈值 (g)，低于此值认为设备未运行，默认 0.005
} rpm_calib_config_t;

/**
 * @brief 获取默认 RPM 标定配置
 * 
 * @return rpm_calib_config_t 默认配置
 */
rpm_calib_config_t algo_rpm_calib_get_default_config(void);

/**
 * @brief 自动轴选方法：从三轴加速度数据计算转速
 * 
 * 自动探测振幅最大的轴，然后进行转速标定。
 * 适用于轴向不确定的场景。
 * 
 * @param x_data X 轴数据 (g)，长度 = fft_size
 * @param y_data Y 轴数据 (g)，长度 = fft_size
 * @param z_data Z 轴数据 (g)，长度 = fft_size
 * @param expected_rpm 期望转速 (RPM)，若 > 0 则用窄窗搜索，否则全频段盲搜
 * @param config 标定配置参数指针，若为 NULL 则使用默认配置
 * @param out_rpm 输出计算得到的转速指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 */
esp_err_t algo_rpm_calibration_auto_axis(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm);

/**
 * @brief 频谱搜索方法：从预准备好的 FFT 幅值谱寻找 RPM 峰值
 * 
 * 适用于已有 FFT 结果的场景，避免重复计算 FFT。
 * 
 * @param fft_mag FFT 幅值谱 (长度 = fft_size/2)
 * @param expected_rpm 期望转速 (RPM)，若 > 0 则用窄窗搜索，否则全频段盲搜
 * @param config 标定配置参数指针，若为 NULL 则使用默认配置
 * @param out_rpm 输出计算得到的转速指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 */
esp_err_t algo_rpm_calibration_spectrum_search(
    const float *fft_mag,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm);

/**
 * @brief 基于谐波评分的 RPM 标定结果
 */
typedef struct {
    float rpm;              ///< 计算得到的转速 (RPM)
    float freq_hz;          ///< 对应的频率 (Hz)
    float confidence;       ///< 置信度，范围约 [0, 100+]，基于峰值-噪声比
    float window_seconds;   ///< 采集窗口时长 (秒)
    float rayleigh_hz;      ///< 瑞利分辨率 (Hz / bin)
} rpm_harmonic_result_t;

/**
 * @brief 基于物理滑差频带的 RPM 标定结果
 */
typedef struct {
    float rpm;              ///< 计算得到的转速 (RPM)
    float freq_hz;          ///< 对应的频率 (Hz)
    float peak_amplitude;   ///< 检测到的峰值幅度
    float window_seconds;   ///< 采集窗口时长 (秒)
    float rayleigh_hz;      ///< 瑞利分辨率 (Hz / bin)
} rpm_slip_model_result_t;

/**
 * @brief 谐波评分方法：使用谐波支持度和抛物线插值进行精细转速标定
 * 
 * 评分方式：f + 0.5*2f + 0.25*3f，优先选择谐波结构完整的频率。
 * 高精度转速测量（±0.1 RPM），但计算量较大。
 * 
 * @param signal 单轴原始加速度数据 (g)
 * @param signal_length 数据长度（点数）
 * @param sampling_rate 采样率 (Hz)
 * @param sync_rpm 同步转速（期望值，RPM），用于窄窗搜索。若 <= 0 则全频段搜索
 * @param max_slip_rpm 最大滑差转速 (RPM)，搜索范围为 [sync_rpm-max_slip_rpm, sync_rpm)
 * @param pad_factor 零填充倍数，默认 8。值越大频率分辨率越高，但计算量增加
 * @param out_result 输出结果指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 */
esp_err_t algo_rpm_calibration_harmonic_score(
    const float *signal,
    uint32_t signal_length,
    float sampling_rate,
    int32_t sync_rpm,
    int32_t max_slip_rpm,
    uint32_t pad_factor,
    rpm_harmonic_result_t *out_result);

/**
 * @brief 滑差模型方法：基于电机物理滑差频带的高精度 RPM 标定
 * 
 * 利用交流感应电机的物理滑差约束（0~7%）进行精确的动态速度检测。
 * 计算量小，精度高，生产级别推荐方案。
 * 
 * @param signal 单轴原始加速度数据 (g)，通常为轴向数据
 * @param signal_length 数据长度（点数）
 * @param sampling_rate 采样率 (Hz)
 * @param sync_rpm 电机同步转速 (RPM)，如 1800
 * @param max_slip_rpm 最大滑差转速 (RPM)，通常为 100-150
 * @param pad_factor 零填充倍数，建议 5-8
 * @param out_result 输出结果指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 */
esp_err_t algo_rpm_calibration_slip_model(
    const float *signal,
    uint32_t signal_length,
    float sampling_rate,
    int32_t sync_rpm,
    int32_t max_slip_rpm,
    uint32_t pad_factor,
    rpm_slip_model_result_t *out_result);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RPM_CALIBRATION_H
