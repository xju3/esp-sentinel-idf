#include "task_monitor.h"
#include "algo_pdm.h"
#include "algo_stats.h"
#include "daq_icm_42688_p.h"
#include "drv_icm_42688_p.h"
#include "config_manager.h"
#include "logger.h"
// system
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <time.h>
#include <string.h>
#include <math.h>

#define MONITOR_QUEUE_SIZE 10
#define DAQ_CHUNK_SIZE 128

// Monitor -> dispatcher message queue (created in task_monitor_start)
icm_cfg_t cfg;
QueueHandle_t g_monitor_message_queue = NULL;
SemaphoreHandle_t g_wakeup_sem = NULL;
monitor_mode_t g_monitor_mode = MONITOR_MODE_NORMAL;
int8_t warning_times = 0;

static esp_timer_handle_t s_timer = NULL;

// RMS累积器结构体
typedef struct {
    float sum_sq_x;
    float sum_sq_y;
    float sum_sq_z;
    uint32_t count;
} rms_accumulator_t;

// 1. 专属的业务数据处理算子
static void task_monitor_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    rms_accumulator_t *acc = (rms_accumulator_t *)ctx;
    
    LOG_DEBUGF("chunk_handler called: count=%u, ctx=%p", count, ctx);
    LOG_DEBUGF("Before: sum_sq_x=%.4f, sum_sq_y=%.4f, sum_sq_z=%.4f, count=%u", 
               acc->sum_sq_x, acc->sum_sq_y, acc->sum_sq_z, acc->count);
    
    for (size_t i = 0; i < count; i++) {
        // 转换原始数据为加速度值（g）
        float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        
        // 调试：显示第一个样本
        if (i == 0 && acc->count == 0) {
            LOG_DEBUGF("First sample: raw_x=%d, raw_y=%d, raw_z=%d, x_g=%.4f, y_g=%.4f, z_g=%.4f",
                       data[i].x, data[i].y, data[i].z, x_g, y_g, z_g);
        }
        
        // 累积平方和
        acc->sum_sq_x += x_g * x_g;
        acc->sum_sq_y += y_g * y_g;
        acc->sum_sq_z += z_g * z_g;
        acc->count++;
    }
    
    LOG_DEBUGF("After: sum_sq_x=%.4f, sum_sq_y=%.4f, sum_sq_z=%.4f, count=%u", 
               acc->sum_sq_x, acc->sum_sq_y, acc->sum_sq_z, acc->count);
}

static void monitor_timer_cb(void *arg)
{
    if (g_monitor_mode == MONITOR_MODE_NORMAL)
    {
        xSemaphoreGive(g_wakeup_sem);
    }
}

static void monitor_task_loop(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(g_wakeup_sem, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        // 初始化RMS累积器
        rms_accumulator_t accumulator = {0};
        LOG_DEBUG("Starting DAQ capture for RMS calculation");

        // 召唤引擎！把配置、时间和自己的处理函数传进去
        esp_err_t err = daq_icm_42688_p_capture(&cfg, 1000, task_monitor_chunk_handler, &accumulator, DAQ_CHUNK_SIZE);
        if (err == ESP_OK)
        {
            LOG_DEBUGF("DAQ capture completed: sum_sq_x=%.4f, sum_sq_y=%.4f, sum_sq_z=%.4f, count=%u",
                       accumulator.sum_sq_x, accumulator.sum_sq_y, accumulator.sum_sq_z, accumulator.count);
            
            // 计算三轴RMS值
            float rms_x = 0.0f, rms_y = 0.0f, rms_z = 0.0f;
            
            if (accumulator.count > 0) {
                rms_x = sqrtf(accumulator.sum_sq_x / accumulator.count);
                rms_y = sqrtf(accumulator.sum_sq_y / accumulator.count);
                rms_z = sqrtf(accumulator.sum_sq_z / accumulator.count);
            }
            
            // 计算3D RMS: sqrt(RMS_x² + RMS_y² + RMS_z²)
            float rms_3d = sqrtf(rms_x * rms_x + rms_y * rms_y + rms_z * rms_z);
            
            // 创建RMS数据包
            imu_rms_data_t rms_data = {
                .timestamp = esp_timer_get_time(),
                .rms_x = rms_x,
                .rms_y = rms_y,
                .rms_z = rms_z
                // 注意：imu_rms_data_t 结构体没有3D RMS字段
                // 如果需要3D RMS，需要扩展结构体或使用其他方式传递
            };
            
            // 发送到消息队列
            if (g_monitor_message_queue != NULL) {
                if (xQueueSend(g_monitor_message_queue, &rms_data, 0) != pdTRUE) {
                    LOG_WARN("Monitor queue full, dropping RMS data");
                } else {
                    LOG_DEBUGF("RMS calculated: X=%.4fg, Y=%.4fg, Z=%.4fg, 3D=%.4fg, samples=%u", 
                             rms_x, rms_y, rms_z, rms_3d, accumulator.count);
                }
            }
        } else {
            LOG_ERRORF("DAQ capture failed: %d", err);
        }
    }
}

// Control function to pause monitor for FFT diagnosis
void task_monitor_pause_for_fft(void)
{
    g_monitor_mode = MONITOR_MODE_FFT_DIAGNOSIS;
    LOG_DEBUG("Monitor paused for FFT diagnosis");
}

// Control function to resume monitor after FFT diagnosis
void task_monitor_resume_after_fft(void)
{
    g_monitor_mode = MONITOR_MODE_NORMAL;
    LOG_DEBUG("Monitor resumed after FFT diagnosis");
}

esp_err_t task_monitor_start(void)
{
    if (g_monitor_message_queue != NULL)
        return ESP_OK; // Already started
    g_monitor_message_queue = xQueueCreate(MONITOR_QUEUE_SIZE, sizeof(imu_rms_data_t));
    g_wakeup_sem = xSemaphoreCreateBinary();

    // Create hardware timer for periodic wakeup
    esp_timer_create_args_t timer_args = {
        .callback = monitor_timer_cb,
        .name = "monitor_tick"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_timer));

    // 配置传感器参数
    cfg.odr = calculate_patrol_odr(g_user_config.rpm);
    cfg.fs = ICM_FS_16G;
    cfg.enable_wom = false;
    cfg.wom_thr_mg = 0;

    // Start periodic timer (interval in minutes from config)
    int32_t interval_min = g_user_config.detect;
    if (interval_min <= 0)
        interval_min = 1; // Default to 1 min if invalid
    uint64_t interval_us = (uint64_t)interval_min * 10ULL * 1000000ULL;
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, interval_us));

    // Create monitor task
    xTaskCreate(monitor_task_loop, "monitor_task", 4096, NULL, 5, NULL);

    // Trigger first measurement immediately
    xSemaphoreGive(g_wakeup_sem);

    return ESP_OK;
}