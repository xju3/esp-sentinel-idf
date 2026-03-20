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
    sensor_position_t pos;   // 当前安装方向映射
    patrol_peak_t peaks[PATROL_MAX_PEAKS];  // 5 个代表性物理频点，保留三轴幅值分布

    int32_t fault_code;      // 0=无故障,1=不平衡,2=不对中,3=松动,4=轴承,5=电气
    char fault_desc[64];     // 描述性文本（UTF-8）
    float confidence;        // 置信度 [0.0, 1.0]
} patrol_fft_report_t;

/**
 * @brief 初始化 FFT 查找表
 * 在首次调用计算前会自动调用，也可以手动调用以预分配内存
 */
esp_err_t algo_fft_init(void);

/**
 * @brief 对单轴数据执行 FFT 并计算幅值谱 (Amplitude Spectrum)
 * 
 * @param input 输入时域信号 (实数)，长度必须为 n
 * @param output 输出频域幅值 (实数)，长度必须为 n/2
 * @param n FFT 点数 (必须是 2 的幂次方，当前最大支持 4096)
 * @return esp_err_t ESP_OK 成功，其他为错误码
 */
esp_err_t algo_fft_calculate(const float *input, float *output, uint32_t n);

/**
 * @brief 对三轴振动数据执行 FFT 并提取巡检峰值
 * 
 * @param x_data X轴时域信号
 * @param y_data Y轴时域信号
 * @param z_data Z轴时域信号
 * @param n FFT点数 (必须是2的幂次方)
 * @param sample_rate 采样率 (Hz)
 * @param report 输出报告结构体
 * @return esp_err_t ESP_OK 成功，其他为错误码
 */
esp_err_t algo_fft_calculate_peaks(
    const float *x_data,
    const float *y_data,
    const float *z_data,
    uint32_t n,
    float sample_rate,
    patrol_fft_report_t *report);

#ifdef __cplusplus
}
#endif

#endif // ALGO_FFT_H
