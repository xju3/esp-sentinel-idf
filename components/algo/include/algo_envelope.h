#ifndef ALGO_ENVELOPE_H
#define ALGO_ENVELOPE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 轴承故障特征频率系数 (Orders)
 * 这些系数乘以转频 (RPM/60) 即为故障频率
 */
typedef struct {
    float bpfo; // 外圈故障系数 (Ball Pass Frequency Outer)
    float bpfi; // 内圈故障系数 (Ball Pass Frequency Inner)
    float bs;   // 滚动体故障系数 (Ball Spin)
    float ftf;  // 保持架故障系数 (Fundamental Train Frequency)
} bearing_orders_t;

/**
 * @brief 单项故障检测结果
 */
typedef struct {
    bool detected;          // 是否检出
    float amplitude;        // 特征频率处的幅值
    float frequency_hz;     // 实际匹配到的频率 (Hz)
    float confidence;       // 置信度 (0.0 - 1.0)
} fault_stat_t;

/**
 * @brief 包络诊断综合报告
 */
typedef struct {
    float peak_freq_hz;     // 谱中最大峰值频率
    float peak_mag;         // 谱中最大峰值幅值
    
    // 各类故障检测结果
    fault_stat_t bpfo_stat;
    fault_stat_t bpfi_stat;
    fault_stat_t bs_stat;
    fault_stat_t ftf_stat;
} envelope_report_t;

/**
 * @brief 初始化包络算法所需的资源 (如滤波器系数表)
 * @return esp_err_t
 */
esp_err_t algo_envelope_init(void);

/**
 * @brief 执行包络分析与诊断
 * 
 * @param input_data    原始振动数据 (将被修改用于中间计算)
 * @param len           数据长度
 * @param sample_rate   采样率 (Hz)
 * @param rpm           当前转速 (RPM)，用于计算故障频率
 * @param bearing       轴承参数，若为 NULL 则跳过故障匹配
 * @param out_report    输出诊断报告
 * @return esp_err_t 
 */
esp_err_t algo_envelope_execute(float *input_data, uint32_t len, float sample_rate, 
                                float rpm, const bearing_orders_t *bearing,
                                envelope_report_t *out_report);

#ifdef __cplusplus
}
#endif

#endif // ALGO_ENVELOPE_H