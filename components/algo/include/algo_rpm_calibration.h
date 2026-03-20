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
 * @brief 从三轴加速度数据计算转速
 * 
 * @param x_data 优先轴 X 数据 (g)，长度 = fft_size
 * @param y_data 优先轴 Y 数据 (g)，长度 = fft_size
 * @param z_data 优先轴 Z 数据 (g)，长度 = fft_size
 * @param expected_rpm 期望转速 (RPM)，若 > 0 则用窄窗搜索，否则全频段盲搜
 * @param config 标定配置参数指针，若为 NULL 则使用默认配置
 * @param out_rpm 输出计算得到的转速指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 * 
 * @note
 *   - 该函数会自动选择振幅最大的轴（优势轴）进行 FFT 分析
 *   - 若 expected_rpm > 0，搜索范围为 [center_freq*0.7, center_freq*1.3]，可区分基频与谐波
 *   - 输出的转速有效性需通过返回值判断
 */
esp_err_t algo_rpm_calibration_calculate(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm);

/**
 * @brief 从预准备好的 FFT 幅值谱寻找 RPM 峰值
 * 
 * 该函数用于已有 FFT 结果的场景（例如已在其他函数中计算过 FFT）
 * 
 * @param fft_mag FFT 幅值谱 (长度 = fft_size/2)
 * @param expected_rpm 期望转速 (RPM)，若 > 0 则用窄窗搜索，否则全频段盲搜
 * @param config 标定配置参数指针，若为 NULL 则使用默认配置
 * @param out_rpm 输出计算得到的转速指针
 * @return esp_err_t ESP_OK 成功，否则返回对应的错误码
 */
esp_err_t algo_rpm_calibration_from_spectrum(
    const float *fft_mag,
    int32_t expected_rpm,
    const rpm_calib_config_t *config,
    int32_t *out_rpm);

#ifdef __cplusplus
}
#endif

#endif // ALGO_RPM_CALIBRATION_H
