#ifndef TASK_FFT_H
#define TASK_FFT_H

#include "esp_err.h"
#include "task_rms.h" // for vib_job_t

#ifdef __cplusplus
extern "C"
{
#endif

extern QueueHandle_t g_fft_job_queue;

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
    patrol_peak_t peaks[5];  // 5 个代表性物理频点，保留三轴幅值分布
} patrol_fft_report_t;

esp_err_t start_fft_task(void);

#ifdef __cplusplus
}
#endif // TASK_FFT_H
#endif // TASK_FFT_H
