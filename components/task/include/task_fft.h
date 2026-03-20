#ifndef TASK_FFT_H
#define TASK_FFT_H

#include "esp_err.h"
#include "task_rms.h" // for vib_job_t

#ifdef __cplusplus
extern "C"
{
#endif

extern QueueHandle_t g_fft_job_queue;

typedef struct {
    float freq;  // 频率点 (Hz)
    float amp;   // 该点的幅值 (Magnitude)
} peak_point_t;

typedef struct __attribute__((packed)) {
    int32_t task_id;        // 关联 RMS 任务，方便服务器做 Data Fusion
    peak_point_t peaks[5];  // 只抓最强的 5 个“显眼包”
} patrol_fft_report_t;

esp_err_t start_fft_task(void);

#ifdef __cplusplus
}
#endif // TASK_FFT_H
#endif // TASK_FFT_H
