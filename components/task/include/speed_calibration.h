#ifndef SPEED_CALIBRATION_H
#define SPEED_CALIBRATION_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动转速标定流程
 * 
 * 该函数会阻塞当前任务约 10-12 秒，进行低频数据采集和 FFT 分析。
 * 
 * @param[in] expected_rpm 期望转速/铭牌转速 (RPM)。若为 0 则进行全范围盲搜；若 >0 则在 ±30% 范围内搜索以排除谐波干扰。
 * @param[out] out_rpm 输出标定后的转速 (RPM)
 * @return esp_err_t 
 *      - ESP_OK: 标定成功
 *      - ESP_ERR_TIMEOUT: 采集超时
 *      - ESP_ERR_NO_MEM: 内存不足
 *      - ESP_FAIL: 未找到有效的转速峰值
 */
esp_err_t speed_calibration_start(int32_t expected_rpm, int32_t *out_rpm);

#ifdef __cplusplus
}
#endif

#endif // SPEED_CALIBRATION_H