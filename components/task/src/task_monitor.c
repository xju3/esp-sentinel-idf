#include "task_monitor.h"
#include "algo_pdm.h"
#include "algo_stash.h"
#include "algo_welford.h"
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
#include <stdbool.h>

#define MONITOR_QUEUE_SIZE 10
#define DAQ_CHUNK_SIZE 64
#define SAMPLE_DURATION_MS 1000
typedef struct
{
    icm_cfg_t *cfg;
    imu_rms_data_t rms;
    esp_timer_handle_t s_timer;
    vib_welford_3d_t welford_st;
} task_monitor_params_t;

QueueHandle_t g_monitor_message_queue = NULL;
SemaphoreHandle_t g_wakeup_sem = NULL;
monitor_mode_t g_monitor_mode = MONITOR_MODE_PATROLLING;

static void monitor_timer_cb(void *arg)
{
    if (g_monitor_mode == MONITOR_MODE_PATROLLING)
    {
        xSemaphoreGive(g_wakeup_sem);
    }
}

/**
 * @brief 清理监控任务资源
 * @param params 任务参数指针
 * @note 这个函数负责释放所有动态分配的资源
 */
static void cleanup(task_monitor_params_t *params)
{
    if (!params)
    {
        return;
    }

    // 清理定时器
    if (params->s_timer)
    {
        esp_timer_stop(params->s_timer);
        esp_timer_delete(params->s_timer);
        params->s_timer = NULL;
    }

    // 清理传感器配置
    if (params->cfg)
    {
        free(params->cfg);
        params->cfg = NULL;
    }

    // 清理任务参数
    free(params);

    // 清理全局资源
    if (g_wakeup_sem)
    {
        vSemaphoreDelete(g_wakeup_sem);
        g_wakeup_sem = NULL;
    }

    if (g_monitor_message_queue)
    {
        vQueueDelete(g_monitor_message_queue);
        g_monitor_message_queue = NULL;
    }

    LOG_DEBUG("Monitor resources cleaned up");
}

static void monitor_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    vib_welford_3d_t *w = (vib_welford_3d_t *)ctx;
    if (!w || !data)
    {
        return;
    }
    for (size_t i = 0; i < count; i++)
    {
        const float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        const float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        const float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        vib_welford_3d_update(w, x_g, y_g, z_g);
    }
}

static void monitor_task_loop(void *arg)
{
    task_monitor_params_t *params = (task_monitor_params_t *)arg;

    while (1)
    {
        if (xSemaphoreTake(g_wakeup_sem, portMAX_DELAY) != pdTRUE)
            continue;

        disable_icm42688p_wom();
        vib_welford_3d_init(&params->welford_st);

        esp_err_t err = daq_icm_42688_p_capture(params->cfg,
                                                SAMPLE_DURATION_MS,
                                                monitor_chunk_handler,
                                                &params->welford_st,
                                                DAQ_CHUNK_SIZE,
                                                0);

        if (err != ESP_OK)
        {
            LOG_WARN("dma data capture failed.");
            // 【修复】采样失败也要重新开 WoM，否则系统陷入死寂
            enable_icm42688p_wom(124);

            continue;
        }

        vib_welford_feature_out_t feat = {0};
        vib_welford_baseline_rms_from_stats(&params->welford_st, &feat);
        LOG_DEBUGF("samples=%d, x=%.3f, y=%.3f, z=%.3f",
                   (int)params->welford_st.x.n, feat.fx, feat.fy, feat.fz);

        // 【修复】采样结束到 WoM 启用之间，给 MEMS 和信号路径一个缓冲
        // daq_capture 内部调用了 stop_stream，但 ICM FIFO 仍可能有残留
        vTaskDelay(pdMS_TO_TICKS(10)); // 让 drv 层 DMA 任务完全退出后再操作寄存器
        enable_icm42688p_wom(124);
        // 【修复】消除 enable 过程中产生的幽灵中断
        // 先把信号量里可能已经被塞入的幽灵 token 抽干
        // 清空 enable 过程中产生的幽灵中断
        while (xSemaphoreTake(g_wakeup_sem, 0) == pdTRUE)
        {
            LOG_DEBUG("Discarding ghost wakeup signal");
        }
        xTaskNotifyStateClear(imu_task_handle);
    }
}

// Control function to pause monitor for FFT diagnosis
void task_monitor_pause(void)
{
    g_monitor_mode = MONITOR_MODE_DIAGNOSIS;
    LOG_DEBUG("Monitor paused for FFT diagnosis");
}

// Control function to resume monitor after FFT diagnosis
void task_monitor_resume(void)
{
    g_monitor_mode = MONITOR_MODE_PATROLLING;
    LOG_DEBUG("Monitor resumed after FFT diagnosis");
}

esp_err_t task_monitor_start(void)
{
    if (g_monitor_message_queue != NULL)
    {
        LOG_WARN("montor task already started");
        return ESP_OK; // Already started
    }

    // 动态分配任务参数
    task_monitor_params_t *params = (task_monitor_params_t *)malloc(sizeof(task_monitor_params_t));
    if (!params)
    {
        LOG_ERROR("Failed to allocate memory for monitor task parameters");
        return ESP_ERR_NO_MEM;
    }

    // 动态分配传感器配置
    icm_cfg_t *cfg = (icm_cfg_t *)malloc(sizeof(icm_cfg_t));
    if (!cfg)
    {
        LOG_ERROR("Failed to allocate memory for sensor configuration");
        free(params);
        return ESP_ERR_NO_MEM;
    }

    // 初始化参数
    memset(params, 0, sizeof(task_monitor_params_t));
    params->s_timer = NULL;
    params->cfg = cfg;

    // 初始化传感器配置
    cfg->odr = calculate_patrol_odr(g_user_config.rpm);
    cfg->fs = ICM_FS_16G;
    cfg->enable_wom = false;
    cfg->wom_thr_mg = 0;

    // 初始化Welford统计结构
    vib_welford_3d_init(&params->welford_st);

    // 创建消息队列
    g_monitor_message_queue = xQueueCreate(MONITOR_QUEUE_SIZE, sizeof(imu_rms_data_t));
    if (!g_monitor_message_queue)
    {
        LOG_ERROR("Failed to create monitor message queue");
        cleanup(params);
        return ESP_FAIL;
    }

    // 创建唤醒信号量
    g_wakeup_sem = xSemaphoreCreateBinary();
    if (!g_wakeup_sem)
    {
        LOG_ERROR("Failed to create wakeup semaphore");
        cleanup(params);
        return ESP_FAIL;
    }

    // 创建定时器
    esp_timer_create_args_t timer_args = {
        .callback = monitor_timer_cb,
        .name = "monitor_tick"};
    esp_err_t err = esp_timer_create(&timer_args, &params->s_timer);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to create monitor timer: %s", esp_err_to_name(err));
        cleanup(params);
        return err;
    }

    // 配置定时器间隔（从配置中读取分钟数）
    int16_t interval_min = g_user_config.detect;
    if (interval_min <= 0)
        interval_min = 1; // 如果无效，默认1分钟
    uint64_t interval_us = (uint64_t)interval_min * 60ULL * 1000ULL;
    err = esp_timer_start_periodic(params->s_timer, interval_us);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to start monitor timer: %s", esp_err_to_name(err));
        cleanup(params);
        return err;
    }

    // 创建监控任务
    BaseType_t task_result = xTaskCreate(monitor_task_loop, "monitor_task", 4096, params, 5, NULL);
    if (task_result != pdPASS)
    {
        LOG_ERROR("Failed to create monitor task");
        cleanup(params);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    // 发送初始信号唤醒任务
    xSemaphoreGive(g_wakeup_sem);
    LOG_DEBUG("Monitor task started successfully");
    return ESP_OK;
}
