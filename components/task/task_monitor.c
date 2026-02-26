#include "task_monitor.h"
#include "algo_pdm.h"
#include "config_manager.h"
#include "logger.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <string.h>
#include "drv_icm_42688_p.h"
#include "esp_log.h"
#include "daq_icm_42688_p.h"

#define MONITOR_QUEUE_SIZE 10
#define DMA_STREAM_BUFFER_SIZE 16384
#define SAMPLE_DURATION_MS 1000
// Upper bound of samples we keep for ISO10816 processing (1s at 4 kHz)
#define MONITOR_MAX_SAMPLES 4096
#define DAQ_CHUNK_SIZE 128


// Monitor -> dispatcher message queue (created in task_monitor_start)
QueueHandle_t g_monitor_message_queue = NULL;

static SemaphoreHandle_t s_wakeup_sem = NULL;
static esp_timer_handle_t s_timer = NULL;


// 专属的日常巡检处理算子
static void monitor_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    // vib_welford_3d_t *current_stats = (vib_welford_3d_t *)ctx;
    // ... 大小端转换并塞入 current_stats ...
}

static void monitor_timer_cb(void *arg)
{
    xSemaphoreGive(s_wakeup_sem);
}

static void monitor_task_loop(void *arg)
{
    icm_cfg_t cfg = {
        .odr = ICM_ODR_1KHZ,
        .fs = ICM_FS_16G,
        .enable_wom = false,
        .wom_thr_mg = 0};
    algo_welford_ctx_t welford_ctx;
    // Initialize welford context (zero it out)
    welford_ctx.count = 0;
    welford_ctx.mean = 0.0;
    welford_ctx.m2 = 0.0;
    welford_ctx.min_val = 0.0f;
    welford_ctx.max_val = 0.0f;
    
    while (1)
    {
        if (xSemaphoreTake(s_wakeup_sem, portMAX_DELAY) == pdTRUE)
        {
            // TODO: Implement actual data processing
            // For now, just reset the context
            welford_ctx.count = 0;
            welford_ctx.mean = 0.0;
            welford_ctx.m2 = 0.0;
            welford_ctx.min_val = 0.0f;
            welford_ctx.max_val = 0.0f;
            
            // Get statistics
            float mean, variance, std_dev;
            algo_welford_get_stats(&welford_ctx, &mean, &variance, &std_dev);
        }
    }
}

esp_err_t task_monitor_start(void)
{
    if (g_monitor_message_queue != NULL)
        return ESP_OK; // Already started
    g_monitor_message_queue = xQueueCreate(MONITOR_QUEUE_SIZE, sizeof(monitor_msg_t));
    s_wakeup_sem = xSemaphoreCreateBinary();

    // Create hardware timer for periodic wakeup
    esp_timer_create_args_t timer_args = {
        .callback = monitor_timer_cb,
        .name = "monitor_tick"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_timer));

    // Start periodic timer (interval in minutes from config)
    int32_t interval_min = g_user_config.detect;
    if (interval_min <= 0)
        interval_min = 1; // Default to 1 min if invalid
    uint64_t interval_us = (uint64_t)interval_min * 10ULL * 1000000ULL;
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, interval_us));

    // // Create monitor task
    xTaskCreate(monitor_task_loop, "monitor_task", 4096, NULL, 5, NULL);

    // Trigger first measurement immediately
    xSemaphoreGive(s_wakeup_sem);

    return ESP_OK;
}