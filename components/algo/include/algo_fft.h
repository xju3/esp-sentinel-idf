#ifndef ALGO_FFT_H
#define ALGO_FFT_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATROL_MAX_PEAKS 5
#define PATROL_MAX_FREQ_HZ 1000.0f

typedef struct __attribute__((packed)) {
    float freq_hz;        // 峰值频率点 (Hz)
    float amp_x;          // X 轴该频点幅值
    float amp_y;          // Y 轴该频点幅值
    float amp_z;          // Z 轴该频点幅值
    uint8_t dominant_axis; // 主导方向: 0=X, 1=Y, 2=Z
} patrol_peak_t;

typedef struct __attribute__((packed)) {
    int32_t task_id;         // 关联 RMS 任务，方便服务器做 Data Fusion
    int32_t timestamp;       // 报告生成时间
    float sample_rate;       // 频谱对应采样率
    patrol_peak_t peaks[PATROL_MAX_PEAKS];  // 5 个代表性物理频点，保留三轴幅值分布

    int32_t fault_code;      // 0=无故障,1=不平衡,2=不对中,3=松动,4=轴承,5=电气
    char fault_desc[64];     // 描述性文本（UTF-8）
    float confidence;        // 置信度 [0.0, 1.0]
} patrol_fft_report_t;

/**
 * @brief 初始化 FFT 查找表
 * 在首次调用计算前会自动调用。FFT 表按实际点数延迟初始化。
 */
esp_err_t algo_fft_init(void);

/**
 * @brief 执行 N 点实数 FFT 并计算单边幅值谱
 *
 * @param input  输入时域信号，长度为 n (实数)
 * @param output 输出幅值谱，长度至少为 n/2 (实数)
 * @param work_buf 内部计算使用的暂存区，长度必须至少为 n (实数)
 * @param n      FFT 点数，必须是 2 的幂次方且不能为 0
 * @return esp_err_t ESP_OK 成功，ESP_ERR_INVALID_ARG 参数错误
 */
esp_err_t algo_fft_calculate(const float *input, float *output, float *work_buf, uint32_t n);

/**
 * @brief 从三轴幅值谱中提取巡检峰值
 *
 * @param mag_x X轴幅值谱 (长度 n/2)
 * @param mag_y Y轴幅值谱 (长度 n/2)
 * @param mag_z Z轴幅值谱 (长度 n/2)
 * @param fft_len FFT点数 (必须是2的幂次方)
 * @param sample_rate 采样率 (Hz)
 * @param peaks_out 输出峰值数组 (PATROL_MAX_PEAKS)
 * @return esp_err_t ESP_OK 成功，其他为错误码
 */
esp_err_t algo_fft_extract_peaks(
    const float *mag_x,
    const float *mag_y,
    const float *mag_z,
    uint32_t fft_len,
    float sample_rate,
    patrol_peak_t *peaks_out);



#ifdef __cplusplus
}
#endif

#endif // ALGO_FFT_H
