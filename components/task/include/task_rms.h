#ifndef TASK_RMS_H
#define TASK_RMS_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "task_stash.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    float *raw_data;      // 指向静态 Buffer 的指针 (平面化布局: X, Y, Z)
    uint32_t length;      // 单轴有效数据点数
    float sample_rate;    // 实际采样率
    task_mode_t task_mode;// 任务模式 (Patrol/Diagnosis)
} vib_job_t;

extern QueueHandle_t g_rms_job_queue;

esp_err_t start_rms_task(void);

#ifdef __cplusplus
}
#endif // TASK_RMS_H
#endif // TASK_RMS_H