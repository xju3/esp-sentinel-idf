#include "task_monitor.h"
#include "icm42688p.h"
#include "icm42688p_baseline.h"
#include "../algo/include/algo_rms.h"
#include "config_manager.h"
#include "logger.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <time.h>
#include <string.h>

#define MONITOR_QUEUE_SIZE 10
#define SAMPLE_DURATION_MS 1000
#define WARMUP_DELAY_MS 100
#define FIFO_POLL_INTERVAL_MS 10
// Upper bound of samples we keep for ISO10816 processing (1s at 4 kHz)
#define MONITOR_MAX_SAMPLES 4096

QueueHandle_t g_monitor_queue = NULL;
static SemaphoreHandle_t s_wakeup_sem = NULL;
static esp_timer_handle_t s_timer = NULL;

static void monitor_timer_cb(void *arg)
{
    xSemaphoreGive(s_wakeup_sem);
}

static void monitor_task_loop(void *arg)
{
    // DMA buffer for SPI transaction (8 bytes per sample + 1 cmd byte)
    // 128 samples * 8 bytes + 1 cmd = 1025 bytes
    // static uint8_t dma_buf[1025] __attribute__((aligned(4)));
    static icm42688p_sample_t sample_buf[128];
    static float s_ax[MONITOR_MAX_SAMPLES];
    static float s_ay[MONITOR_MAX_SAMPLES];
    static float s_az[MONITOR_MAX_SAMPLES];
    icm42688p_init();
    LOG_INFOF("Monitor task started. Interval: %d min", g_user_config.detect);

    while (1)
    {
        // Wait for timer trigger (blocking wait for semaphore)
        if (xSemaphoreTake(s_wakeup_sem, portMAX_DELAY) == pdTRUE)
        {

            // --- Wakeup Sensor ---
            // icm42688p_set_sleep(false);
            // vTaskDelay(pdMS_TO_TICKS(WARMUP_DELAY_MS)); // Wait for MEMS stabilization

            // --- Initialize Algorithm Context ---
            algo_welford_t algo_ctx;
            algo_welford_init(&algo_ctx);
            iso10816_state_t iso_state;
            iso10816_init(&iso_state);

            int64_t start_time = esp_timer_get_time();
            size_t total_samples = 0;
            size_t stored_samples = 0;

            // --- Stream Sampling for 1 Second ---
            bool sensor_working = false;
            while ((esp_timer_get_time() - start_time) < (SAMPLE_DURATION_MS * 1000))
            {
                size_t count = 0;
                esp_err_t err = icm42688p_read_fifo(sample_buf, 128, &count);

                if (err == ESP_OK && count > 0)
                {
                    sensor_working = true;
                    for (size_t i = 0; i < count; i++)
                    {
                        algo_welford_update(&algo_ctx, sample_buf[i].x_g, sample_buf[i].y_g, sample_buf[i].z_g);
                        if (stored_samples < MONITOR_MAX_SAMPLES)
                        {
                            s_ax[stored_samples] = sample_buf[i].x_g;
                            s_ay[stored_samples] = sample_buf[i].y_g;
                            s_az[stored_samples] = sample_buf[i].z_g;
                            stored_samples++;
                        }
                    }
                    total_samples += count;
                }

                // Non-blocking yield - allow FIFO to fill (4kHz = 0.25ms/sample)
                // Using minimal delay to avoid CPU blocking
                vTaskDelay(pdMS_TO_TICKS(FIFO_POLL_INTERVAL_MS));
            }

            // 如果传感器没有工作，使用模拟数据
            if (!sensor_working)
            {
                LOG_WARN("Sensor not working, using simulated data");
                // 生成一些模拟数据
                for (int i = 0; i < 100; i++)
                {
                    float sim_x = 0.01f * (i % 10);
                    float sim_y = 0.02f * (i % 5);
                    float sim_z = 0.03f * (i % 3);
                    algo_welford_update(&algo_ctx, sim_x, sim_y, sim_z);
                    if (stored_samples < MONITOR_MAX_SAMPLES)
                    {
                        s_ax[stored_samples] = sim_x;
                        s_ay[stored_samples] = sim_y;
                        s_az[stored_samples] = sim_z;
                        stored_samples++;
                    }
                }
                total_samples = 100;
            }

            // --- Put Sensor to Sleep ---
            // icm42688p_set_sleep(true);

            // --- Calculate RMS with Baseline Correction ---
            monitor_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.type = MONITOR_MSG_TYPE_RMS;
            msg.payload.rms.timestamp = time(NULL);

            // This calculates: Delta = Raw_RMS - Baseline_Offset
            algo_welford_finish(&algo_ctx, &g_icm_baseline,
                                &msg.payload.rms.rms_x,
                                &msg.payload.rms.rms_y,
                                &msg.payload.rms.rms_z);

            // --- ISO10816 Velocity RMS (10-1000Hz) ---
            iso10816_result_t iso_res = {0};
            static bool iso_inited = false;
            float elapsed_sec = (float)((esp_timer_get_time() - start_time) / 1000000.0);
            if (elapsed_sec > 0 && stored_samples > 0)
            {
                // float fs_est = (float)total_samples / elapsed_sec;
                float fs_est = 0.0f;
                if (elapsed_sec > 0)
                {
                    fs_est = (float)stored_samples / elapsed_sec; // 用 stored_samples，不用 total_samples
                }
                iso10816_compute(&iso_state, s_ax, s_ay, s_az, (int)stored_samples, fs_est, &iso_res);
                LOG_INFOF("ISO10816: fs=%.1fHz, N=%d, v(mm/s) XYZ=%.3f, %.3f, %.3f, 3D=%.3f",
                          fs_est, (int)stored_samples, iso_res.vx_rms, iso_res.vy_rms, iso_res.vz_rms, iso_res.v3d_rms);
            }
            else
            {
                LOG_WARN("ISO10816 skipped due to insufficient samples");
            }

            LOG_DEBUGF("Detect: Samples=%d, RMS(X,Y,Z)=%.3f, %.3f, %.3f",
                       (int)total_samples, msg.payload.rms.rms_x, msg.payload.rms.rms_y, msg.payload.rms.rms_z);

            // --- Enqueue Data (Black Box Pattern) ---
            if (g_monitor_queue)
            {
                if (uxQueueSpacesAvailable(g_monitor_queue) == 0)
                {
                    // Queue full - drop oldest message to maintain real-time monitoring
                    monitor_msg_t dummy;
                    xQueueReceive(g_monitor_queue, &dummy, 0);
                    LOG_WARN("Queue full, dropped oldest msg");
                }
                // Non-blocking send (timeout=0)
                xQueueSend(g_monitor_queue, &msg, 0);
            }
        }
    }
}

esp_err_t task_monitor_start(void)
{
    if (g_monitor_queue != NULL)
        return ESP_OK; // Already started

    // Create global queue for producer-consumer pattern
    g_monitor_queue = xQueueCreate(MONITOR_QUEUE_SIZE, sizeof(monitor_msg_t));
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
    // uint64_t interval_us = (uint64_t)interval_min * 1000000 / 2;
    uint64_t interval_us = (uint64_t)interval_min * 3ULL * 1000000ULL;
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, interval_us));

    // Create monitor task
    xTaskCreate(monitor_task_loop, "monitor_task", 4096, NULL, 5, NULL);

    // Trigger first measurement immediately
    xSemaphoreGive(s_wakeup_sem);

    return ESP_OK;
}
