#include "task_monitor.h"
#include "algo_rms.h"
#include "config_manager.h"
#include "logger.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <string.h>
#include "drv_icm_42688_p.h"
#include "esp_log.h"

#define MONITOR_QUEUE_SIZE 10
#define SAMPLE_DURATION_MS 1000
#define WARMUP_DELAY_MS 100
#define FIFO_POLL_INTERVAL_MS 10
// Upper bound of samples we keep for ISO10816 processing (1s at 4 kHz)
#define MONITOR_MAX_SAMPLES 4096

QueueHandle_t g_monitor_queue = NULL;
static SemaphoreHandle_t s_wakeup_sem = NULL;
static esp_timer_handle_t s_timer = NULL;

static void task_monitor_imu_data_callback(const icm_raw_data_t *data, size_t count)
{
    static uint32_t chunk_counter = 0;
    chunk_counter++;
    if (count > 0 && (chunk_counter % 10 == 0)) {
        // ICM 传感器硬件输出是大端(Big-Endian)，ESP32 内存是小端(Little-Endian)
        // 必须进行一次字节翻转才能得到真实的整型数值
        int16_t real_x = (int16_t)__builtin_bswap16((uint16_t)data[0].x);
        int16_t real_y = (int16_t)__builtin_bswap16((uint16_t)data[0].y);
        int16_t real_z = (int16_t)__builtin_bswap16((uint16_t)data[0].z);
        // 顺便把 Header 打出来，如果配置正确，Header 的 Bit6 应该是 1 (代表 Accel)
        ESP_LOGI("IMU_TEST", "Chunk #%lu | Header: 0x%02X | X=%d, Y=%d, Z=%d", 
                 chunk_counter, data[0].header, real_x, real_y, real_z);
    }
}


static void monitor_timer_cb(void *arg)
{
    xSemaphoreGive(s_wakeup_sem);
}

static void monitor_task_loop(void *arg)
{
    while (1)
    {
        // Wait for timer trigger (blocking wait for semaphore)
        if (xSemaphoreTake(s_wakeup_sem, portMAX_DELAY) == pdTRUE)
        {
            LOG_DEBUG("Starting DMA stream at 1kHz...");
            esp_err_t err = drv_icm42688_start_stream(task_monitor_imu_data_callback);
            if (err != ESP_OK)
            {
                LOG_ERROR("Stream start failed!");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
            LOG_DEBUG("5 seconds passed. Stopping DMA stream...");
            drv_icm42688_stop_stream();
            LOG_DEBUG("Test finished successfully. System idle.");
        }
    }
}

esp_err_t task_monitor_start(void)
{
    if (g_monitor_queue != NULL)
        return ESP_OK; // Already started
    LOG_DEBUG("Starting ICM-42688-P DMA Stream Test...");
    // 第一步：基础物理初始化 (总线和 DMA 内存分配)
    esp_err_t err = drv_icm42688_init();
    if (err != ESP_OK)
    {
        LOG_DEBUGF("IMU Init failed! Error: %d", err);
        return ESP_ERR_INVALID_STATE;
    }
    LOG_DEBUG("IMU Init OK.");

    // 第二步：动态配置 (1kHz ODR, 16g 量程, 不开启 WoM 休眠唤醒)
    icm_cfg_t cfg = {
        .odr = ICM_ODR_1KHZ,
        .fs = ICM_FS_16G,
        .enable_wom = false,
        .wom_thr_mg = 0};
    err = drv_icm42688_config(&cfg);
    if (err != ESP_OK)
    {
        LOG_ERROR("IMU Config failed!");
        return ESP_ERR_INVALID_STATE;
    }
    LOG_DEBUG("IMU Config OK.");

    // Create global queue for producer-consumer pattern
    // g_monitor_queue = xQueueCreate(MONITOR_QUEUE_SIZE, sizeof(monitor_accel_raw_data_t));
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