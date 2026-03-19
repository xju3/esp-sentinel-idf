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

    typedef struct
    {
        float rms;
        float peak;
        float crest_factor;   // 峰值因子
        float impulse_factor; // 脉冲指标
    } axis_features_t;

    typedef struct
    {
        int32_t task_id;       // 任务 ID (可用于追踪和日志)
        int32_t timestamp;     // 报告生成时间 (Unix 时间戳)
        float sample_rate;     // 采样率 (Hz)
        int16_t temperature;   // 采集时的温度 (单位: 0.1°C)
        int8_t conclusion;     // 振动状态结论 (0=正常, 1=警告, 2=危险)
        task_mode_t task_mode; // 任务模式 (Patrol/Diagnosis)
        // 三轴独立的特征数据
        axis_features_t x_axis;
        axis_features_t y_axis;
        axis_features_t z_axis;
    } task_rms_report_t;

    extern QueueHandle_t g_rms_job_queue;

    esp_err_t start_rms_task(void);

#ifdef __cplusplus
}
#endif // TASK_RMS_H
#endif // TASK_RMS_H