#include "drv_icm_42688_p.h"
#include "config_manager.h"
#include "daq_worker.h"
#include "logger.h"
// system
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
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

// 任务模式标志
static volatile task_mode_t current_task_mode = TASK_MODE_PATROLING;
static SemaphoreHandle_t task_mode_mutex = NULL;

// FreeRTOS 软件定时器句柄
static TimerHandle_t patrol_timer_handle = NULL;
static TimerHandle_t diagnosis_timer_handle = NULL;

// 任务上下文结构体
typedef struct {
    task_mode_t task_mode;
    const char *task_name;
    bool is_priority;
} task_context_t;

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
        // 等待通知
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            if (ctx->is_priority)
            {
                // 优先级任务（诊断）：设置当前任务模式为诊断
                if (xSemaphoreTake(task_mode_mutex, portMAX_DELAY) == pdTRUE)
                {
                    task_mode_t previous_mode = current_task_mode;
                    current_task_mode = ctx->task_mode;
                    xSemaphoreGive(task_mode_mutex);
                    
                    LOG_INFOF("[TASK_DAQ] %s task triggered (priority)", ctx->task_name);
                    
                    // 准备参数并直接调用 start_daq_worker
                    daq_worker_param_t param = {
                        .rpm = g_user_config.rpm,
                        .task_mode = ctx->task_mode
                    };
                    
                    esp_err_t err = start_daq_worker(&param);
                    if (err != ESP_OK)
                    {
                        LOG_ERRORF("[TASK_DAQ] %s work failed: %s", ctx->task_name, esp_err_to_name(err));
                    }
                    
                    // 恢复之前的任务模式
                    if (xSemaphoreTake(task_mode_mutex, portMAX_DELAY) == pdTRUE)
                    {
                        current_task_mode = previous_mode;
                        xSemaphoreGive(task_mode_mutex);
                    }
                }
            }
            else
            {
                // 非优先级任务（巡逻）：检查当前是否有优先级任务在执行
                if (xSemaphoreTake(task_mode_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    if (current_task_mode == ctx->task_mode)
                    {
                        LOG_INFOF("[TASK_DAQ] %s task triggered", ctx->task_name);
                        
                        // 准备参数并直接调用 start_daq_worker
                        daq_worker_param_t param = {
                            .rpm = g_user_config.rpm,
                            .task_mode = ctx->task_mode
                        };
                        
                        esp_err_t err = start_daq_worker(&param);
                        if (err != ESP_OK)
                        {
                            LOG_ERRORF("[TASK_DAQ] %s work failed: %s", ctx->task_name, esp_err_to_name(err));
                        }
                    }
                    xSemaphoreGive(task_mode_mutex);
                }
            }
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
 * @return true 定时器创建成功
 * @return false 定时器创建失败或间隔为0
 */
static bool init_task_timer(TimerHandle_t *timer_handle_ptr, TaskHandle_t target_task, 
                           int16_t interval_minutes, const char *timer_name)
{
    // 如果间隔为0，则禁用定时器
    if (interval_minutes <= 0)
    {
        LOG_INFOF("[TASK_DAQ] %s timer disabled (interval = %d minutes)", timer_name, interval_minutes);
        *timer_handle_ptr = NULL;
        return false;
    }
    
    // 计算间隔（分钟转换为毫秒）
    uint64_t interval_ms = (uint64_t)interval_minutes * 60 * 1000;
    
    // 检查是否超过最大允许值（portMAX_DELAY ticks）
    TickType_t interval_ticks = pdMS_TO_TICKS(interval_ms);
    if (interval_ticks == portMAX_DELAY)
    {
        LOG_ERRORF("[TASK_DAQ] %s timer interval too large: %d minutes", timer_name, interval_minutes);
        *timer_handle_ptr = NULL;
        return false;
    }
    
    // 创建定时器（自动重载模式）
    TimerHandle_t timer_handle = xTimerCreate(
        timer_name,                     // 定时器名称
        interval_ticks,                 // 周期（ticks）
        pdTRUE,                         // 自动重载
        (void *)target_task,            // 定时器ID（存储目标任务句柄）
        timer_callback                  // 回调函数
    );
    
    if (timer_handle == NULL)
    {
        LOG_ERRORF("[TASK_DAQ] Failed to create %s timer", timer_name);
        *timer_handle_ptr = NULL;
        return false;
    }
    
    // 启动定时器
    if (xTimerStart(timer_handle, 0) != pdPASS)
    {
        LOG_ERRORF("[TASK_DAQ] Failed to start %s timer", timer_name);
        xTimerDelete(timer_handle, 0);
        *timer_handle_ptr = NULL;
        return false;
    }
    
    *timer_handle_ptr = timer_handle;
    LOG_INFOF("[TASK_DAQ] %s timer initialized with interval: %d minutes (%llu ms)", 
              timer_name, interval_minutes, interval_ms);
    return true;
}

 void start_task_daq(void) {
    LOG_INFO("[TASK_DAQ] Starting DAQ tasks...");
    
    // 创建互斥锁
    task_mode_mutex = xSemaphoreCreateMutex();
    if (task_mode_mutex == NULL)
    {
        LOG_ERROR("[TASK_DAQ] Failed to create mutex");
        return;
    }
    
    // 创建任务上下文
    task_context_t patrol_ctx = {
        .task_mode = TASK_MODE_PATROLING,
        .task_name = "Patrol",
        .is_priority = false
    };
    
    task_context_t diagnosis_ctx = {
        .task_mode = TASK_MODE_DIAGNOSIS,
        .task_name = "Diagnosis",
        .is_priority = true
    };
    
    // 创建巡逻任务
    BaseType_t result = xTaskCreate(
        generic_task_handler,
        "patrol_task",
        4096,
        &patrol_ctx,
        2,  // 优先级较低
        &patrol_task_handle
    );
    
    if (result != pdPASS)
    {
        LOG_ERROR("[TASK_DAQ] Failed to create patrol task");
        vSemaphoreDelete(task_mode_mutex);
        return;
    }
    
    // 创建诊断任务
    result = xTaskCreate(
        generic_task_handler,
        "diagnosis_task",
        4096,
        &diagnosis_ctx,
        3,  // 优先级较高
        &diagnosis_task_handle
    );
    
    if (result != pdPASS)
    {
        LOG_ERROR("[TASK_DAQ] Failed to create diagnosis task");
        vTaskDelete(patrol_task_handle);
        vSemaphoreDelete(task_mode_mutex);
        return;
    }
    
    // 初始化巡逻定时器（分钟级）
    if (!init_task_timer(&patrol_timer_handle, patrol_task_handle, 
                        g_user_config.patrol, "Patrol"))
    {
        LOG_WARN("[TASK_DAQ] Patrol timer not started");
    }
    
    // 初始化诊断定时器（分钟级）
    if (!init_task_timer(&diagnosis_timer_handle, diagnosis_task_handle, 
                        g_user_config.diagnosis, "Diagnosis"))
    {
        LOG_WARN("[TASK_DAQ] Diagnosis timer not started");
    }
    
    LOG_INFO("[TASK_DAQ] DAQ tasks started successfully");
 }
        
