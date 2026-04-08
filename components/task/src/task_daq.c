#include "task_daq.h"
#include "config_manager.h"
#include "drv_icm_42688_p.h"
#include "daq_worker.h"
#include "logger.h"
#include "machine_state.h"
// system
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/idf_additions.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include <time.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/**
 * @file task_daq.c
 * @author Aeons Team
 * @date 2024-06-01
 * @version 1.
 * @brief 任务：定时采样，数据分发及异常检测
 *
 * 该模块负责定时采集数据，并将数据分发给两路任务：
 * 1. Welford算法与RMS检查机械状态，如转子不平衡, 固定螺丝松𥁧,
 * 2. 进行峭度, 包络解调, FFT, 检测内部的轴承, 叶轮异常
 */

// 任务句柄
static TaskHandle_t patrol_task_handle = NULL;
static TaskHandle_t diagnosis_task_handle = NULL;

// 互斥锁，确保同一时间只有一个采集任务在运行
static SemaphoreHandle_t s_daq_mutex = NULL;
static portMUX_TYPE s_task_state_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_diagnosis_pending = false;
static bool s_diagnosis_running = false;

// FreeRTOS 软件定时器句柄
static TimerHandle_t patrol_timer_handle = NULL;
static TimerHandle_t diagnosis_timer_handle = NULL;
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

// 任务上下文结构体
typedef struct
{
    task_mode_t task_mode;
    const char *task_name;
} task_context_t;

static bool diagnosis_active_or_pending(void)
{
    bool active = false;

    taskENTER_CRITICAL(&s_task_state_lock);
    active = s_diagnosis_pending || s_diagnosis_running;
    taskEXIT_CRITICAL(&s_task_state_lock);

    return active;
}

static void set_diagnosis_pending(bool pending)
{
    taskENTER_CRITICAL(&s_task_state_lock);
    s_diagnosis_pending = pending;
    taskEXIT_CRITICAL(&s_task_state_lock);
}

static void set_diagnosis_running(bool running)
{
    taskENTER_CRITICAL(&s_task_state_lock);
    s_diagnosis_running = running;
    taskEXIT_CRITICAL(&s_task_state_lock);
}

/**
 * @brief 定时器回调函数
 *
 * @param timer_handle 定时器句柄
 * @param target_task 目标任务句柄（通过 pvTimerGetTimerID 获取）
 */
static void timer_callback(TimerHandle_t timer_handle)
{
    TaskHandle_t target_task = (TaskHandle_t)pvTimerGetTimerID(timer_handle);
    if (target_task != NULL)
    {
        if (target_task == diagnosis_task_handle)
        {
            set_diagnosis_pending(true);
        }
        else if (target_task == patrol_task_handle && diagnosis_active_or_pending())
        {
            LOG_INFO("Patrol trigger skipped: diagnosis is pending or running");
            return;
        }

        xTaskNotifyGive(target_task);
    }
}

/**
 * @brief 通用任务处理函数
 */
static void generic_task_handler(void *arg)
{
    task_context_t *ctx = (task_context_t *)arg;

    while (1)
    {
        // 等待定时器通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ctx->task_mode == TASK_MODE_PATROLING && diagnosis_active_or_pending())
        {
            LOG_INFO("Patrol task skipped: diagnosis is pending or running");
            continue;
        }

        // Check machine state before proceeding - only run DAQ in STABLE state
        // if (get_machine_state() != STATE_STABLE) {
        //     LOG_DEBUGF("%s task skipped: machine state is not STABLE", ctx->task_name);
        //     continue;
        // }

        // 获取锁，如果当前有其他任务（如 Patrol）正在运行，Diagnosis 会在此阻塞等待
        // 由于 Diagnosis 任务优先级更高，一旦锁释放，它会优先获得锁并立即执行
        if (xSemaphoreTake(s_daq_mutex, portMAX_DELAY) == pdTRUE)
        {
            if (ctx->task_mode == TASK_MODE_PATROLING && diagnosis_active_or_pending())
            {
                LOG_INFO("Patrol task skipped after lock: diagnosis is pending or running");
                xSemaphoreGive(s_daq_mutex);
                continue;
            }

            if (ctx->task_mode == TASK_MODE_DIAGNOSIS)
            {
                set_diagnosis_pending(false);
                set_diagnosis_running(true);
            }

            // LOG_INFOF("%s task triggered", ctx->task_name);

            daq_worker_param_t param = {
                .rpm = g_user_config.rpm,
                .task_mode = ctx->task_mode};

            esp_err_t err = start_daq_worker(&param);
            if (err != ESP_OK)
            {
                LOG_ERRORF("%s work failed: %s", ctx->task_name, esp_err_to_name(err));
            }

            if (ctx->task_mode == TASK_MODE_DIAGNOSIS)
            {
                set_diagnosis_running(false);
            }

            xSemaphoreGive(s_daq_mutex);
        }
    }
}

/**
 * @brief 初始化任务定时器（分钟级）
 *
 * @param timer_handle_ptr 定时器句柄指针
 * @param target_task 目标任务句柄
 * @param interval_minutes 定时器间隔（分钟）
 * @param timer_name 定时器名称（用于日志）
 * @param accel_rate 加速测试（用于检查电池耗用）
 * @return true 定时器创建成功
 * @return false 定时器创建失败或间隔为0
 */
static bool init_task_timer(TimerHandle_t *timer_handle_ptr,
                            TaskHandle_t target_task,
                            int16_t interval_minutes,
                            const char *timer_name,
                            const int8_t accel_rate)
{
    // 如果间隔为0，则禁用定时器
    if (interval_minutes <= 0)
    {
        LOG_INFOF("%s timer disabled (interval = %d minutes)", timer_name, interval_minutes);
        *timer_handle_ptr = NULL;
        return false;
    }

    // 计算间隔（分钟转换为毫秒）
    uint64_t interval_ms = (uint64_t)interval_minutes * 60 * 1000 / accel_rate;
    LOG_DEBUGF("%s timer interval: %d minutes (%llu ms)",
               timer_name, interval_minutes, interval_ms / 1000);
    // 检查是否超过最大允许值（portMAX_DELAY ticks）
    TickType_t interval_ticks = pdMS_TO_TICKS(interval_ms);

    if (interval_ticks == portMAX_DELAY)
    {
        LOG_ERRORF("%s timer interval too large: %d minutes", timer_name, interval_minutes);
        *timer_handle_ptr = NULL;
        return false;
    }

    // 创建定时器（自动重载模式）
    TimerHandle_t timer_handle = xTimerCreate(
        timer_name,          // 定时器名称
        interval_ticks,      // 周期（ticks）
        pdTRUE,              // 自动重载
        (void *)target_task, // 定时器ID（存储目标任务句柄）
        timer_callback       // 回调函数
    );

    if (timer_handle == NULL)
    {
        LOG_ERRORF("Failed to create %s timer", timer_name);
        *timer_handle_ptr = NULL;
        return false;
    }

    // 启动定时器
    if (xTimerStart(timer_handle, 0) != pdPASS)
    {
        LOG_ERRORF("Failed to start %s timer", timer_name);
        xTimerDelete(timer_handle, 0);
        *timer_handle_ptr = NULL;
        return false;
    }

    *timer_handle_ptr = timer_handle;
    LOG_INFOF("%s timer initialized with interval: %d minutes (%llu ms)",
              timer_name, interval_minutes, interval_ms);
    return true;
}

esp_err_t start_task_daq(void)
{
    // 创建互斥锁 (Mutex)
    s_daq_mutex = xSemaphoreCreateMutex();
    if (s_daq_mutex == NULL)
    {
        LOG_ERROR("Failed to create mutex");
        return ESP_ERR_INVALID_STATE;
    }

    // 创建任务上下文 (必须是 static 或堆内存，因为任务会长期引用)
    static task_context_t patrol_ctx = {
        .task_mode = TASK_MODE_PATROLING,
        .task_name = "Patrol"};

    static task_context_t diagnosis_ctx = {
        .task_mode = TASK_MODE_DIAGNOSIS,
        .task_name = "Diagnosis"};

    // 创建巡逻任务
    BaseType_t result = xTaskCreateWithCaps(
        generic_task_handler,
        "patrol_task",
        4096,
        &patrol_ctx,
        2,
        &patrol_task_handle,
        TASK_MEM_CAPS);

    if (result != pdPASS)
    {
        LOG_ERROR("Failed to create patrol task");
        vSemaphoreDelete(s_daq_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // 创建诊断任务
    result = xTaskCreateWithCaps(
        generic_task_handler,
        "diagnosis_task",
        4096,
        &diagnosis_ctx,
        3,
        &diagnosis_task_handle,
        TASK_MEM_CAPS);

    if (result != pdPASS)
    {
        LOG_ERROR("Failed to create diagnosis task");
        vTaskDeleteWithCaps(patrol_task_handle);
        vSemaphoreDelete(s_daq_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    // 初始化巡逻定时器（分钟级）
    LOG_DEBUGF("Initializing DAQ timers with patrol interval: %d minutes, diagnosis interval: %d minutes",
               g_user_config.patrol, g_user_config.diagnosis);

    if (!init_task_timer(&patrol_timer_handle, patrol_task_handle,
                         g_user_config.patrol, "Patrol", 1))
    {
        LOG_WARN("Patrol timer not started");
    }

    // 初始化诊断定时器（分钟级）
    if (!init_task_timer(&diagnosis_timer_handle, diagnosis_task_handle,
                         g_user_config.diagnosis, "Diagnosis", 1))
    {
        LOG_WARN("Diagnosis timer not started");
    }

    LOG_INFO("DAQ tasks started successfully");

    return ESP_OK;
}
