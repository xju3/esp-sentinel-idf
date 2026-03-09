#ifndef TASK_STASH_H
#define TASK_STASH_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_err.h"
#include <time.h>
#include <string.h>
#include <math.h>

#include "config_manager.h"
#include "daq_icm_42688_p.h"
#include "data_dispatcher.h"
#include "drv_icm_42688_p.h"
#include "logger.h"

#ifdef __cplusplus
extern "C"
{
#endif
#define LSB_TO_G_2G (2.0f / 32768.0f)
#define LSB_TO_G_4G (4.0f / 32768.0f)
#define LSB_TO_G_8G (8.0f / 32768.0f)
#define LSB_TO_G_16G (16.0f / 32768.0f)
    // daq operating modes
    typedef enum
    {
        TASK_MODE_PATROLING, // Normal periodic daqing
        TASK_MODE_DIAGNOSIS  // FFT diagnosis in progress
    } task_mode_t;

    typedef struct
    {
        float val;    // baseline mean（例如：静态零偏均值）
        float offset; // baseline offset（例如：环境噪声RMS偏置）
    } vib_axis_baseline_t;

    typedef struct
    {
        vib_axis_baseline_t x;
        vib_axis_baseline_t y;
        vib_axis_baseline_t z;
    } vib_baseline_t;

    typedef struct
    {
        float *raw_data;       // 指向静态 Buffer 的指针 (平面化布局: X, Y, Z)
        uint32_t length;       // 单轴有效数据点数
        float sample_rate;     // 实际采样率
        task_mode_t task_mode; // 任务模式 (Patrol/Diagnosis)
    } vib_job_t;

#ifdef __cplusplus
}
#endif

#endif // TASK_STASH_H