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
#include <stdbool.h>

#define MONITOR_QUEUE_SIZE 10
#define DAQ_CHUNK_SIZE 128
#define MAX_WARNING 5
#define WELFORD_WARNING_THRESHOLD 0.1f

typedef struct
{
    icm_cfg_t *cfg;
    imu_rms_data_t rms;
    float welford_threshold;
    int8_t warning_count;
    esp_timer_handle_t s_timer;
    vib_welford_3d_t welford_st;
} task_monitor_params_t;

QueueHandle_t g_monitor_message_queue = NULL;
SemaphoreHandle_t g_wakeup_sem = NULL;
monitor_mode_t g_monitor_mode = MONITOR_MODE_PATROLLING;

static void monitor_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    vib_welford_3d_t *welford_st = (vib_welford_3d_t *)ctx;
    for (size_t i = 0; i < count; i++)
    {
        float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        // LOG_DEBUGF("x=%f, y=%f, z=%f", x_g, y_g, z_g);
        vib_welford_3d_update(welford_st, x_g, y_g, z_g);
    }
}

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

static void monitor_task_loop(void *arg)
{
    task_monitor_params_t *params = (task_monitor_params_t *)arg;
    while (1)
    {
        if (xSemaphoreTake(g_wakeup_sem, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        // 重置Welford统计，开始新的采样周期
        vib_welford_3d_init(&params->welford_st);

        // 召唤引擎！把配置、时间和自己的处理函数传进去
        esp_err_t err = daq_icm_42688_p_capture(params->cfg, 1000, monitor_chunk_handler, &params->welford_st, DAQ_CHUNK_SIZE);
        if (err == ESP_OK)
        {
            params->rms.rms_x = vib_welford_1d_mean(&params->welford_st.x);
            params->rms.rms_y = vib_welford_1d_mean(&params->welford_st.y);
            params->rms.rms_z = vib_welford_1d_mean(&params->welford_st.z);
            
            LOG_DEBUGF("rms(g) origin: 3d=%f, x=%f, y=%f, z=%f",
                       params->rms.rms_3d, params->rms.rms_x,
                       params->rms.rms_y, params->rms.rms_z);
            
            params->rms.rms_x-= g_baseline.x.val;
            params->rms.rms_y-= g_baseline.y.val;
            params->rms.rms_z-= g_baseline.z.val;

            params->rms.rms_3d = vib_3d_norm(params->rms.rms_x,
                                             params->rms.rms_y,
                                             params->rms.rms_z);
            if (fabsf(params->rms.rms_x) < fabsf(g_baseline.x.offset))
            {
                params->rms.rms_x = 0.0f;
            }

            if (fabsf(params->rms.rms_y) < fabsf(g_baseline.y.offset))
            {
                params->rms.rms_y = 0.0f;
            }

            if (fabsf(params->rms.rms_z) < fabsf(g_baseline.z.offset))
            {
                params->rms.rms_z = 0.0f;
            }

            bool warning = (params->rms.rms_x >= params->welford_threshold ||
                            params->rms.rms_y >= params->welford_threshold ||
                            params->rms.rms_z >= params->welford_threshold);
            if (warning)
            {
                params->warning_count++;
                LOG_WARNF("warning: 3d=%f, x=%f, y=%f, z=%f",
                          params->rms.rms_3d, params->rms.rms_x,
                          params->rms.rms_y, params->rms.rms_z);

                // 检查是否达到最大警告次数
                if (params->warning_count >= MAX_WARNING)
                {
                    LOG_ERROR("Maximum warning count reached, taking action");
                    // TODO: 这里应该触发某种动作，比如发送警报或停止设备
                    params->warning_count = 0; // 重置计数
                }
            }
            else
            {
                // 如果没有警告，逐渐减少计数（但不能低于0）
                if (params->warning_count > 0)
                {
                    params->warning_count--;
                }
            }
        }
        else
        {
            LOG_WARN("dma data capture failed.");
        }
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
    params->welford_threshold = WELFORD_WARNING_THRESHOLD;
    params->warning_count = 0;
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
    uint64_t interval_us = (uint64_t)interval_min * 60ULL * 1000000ULL;
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
