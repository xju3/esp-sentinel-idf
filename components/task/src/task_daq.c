/**
 * @file daq_scheduler.c (由原 task_daq.c 重构而来)
 * @brief 基于深度睡眠和RTC的DAQ一次性调度器
 */
#include "task_daq.h"
#include "config_manager.h"
#include "daq_worker.h"
#include "logger.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include <time.h>

// 允许唤醒时间的误差窗口（秒），应对系统唤醒启动和NTP同步耗时
#define WAKEUP_TOLERANCE_SEC 120

// 使用 RTC_DATA_ATTR 确保在 Deep Sleep 期间调度数据不丢失 (掉电重启才会重置)
RTC_DATA_ATTR static time_t s_next_patrol_time = 0;
RTC_DATA_ATTR static time_t s_next_diagnosis_time = 0;
RTC_DATA_ATTR static bool s_is_cold_boot = true;

/**
 * @brief 从深度睡眠唤醒后调用的 DAQ 评估与执行入口
 * 此函数应当在 app_main 中网络连接与NTP同步完成后被调用
 */
esp_err_t daq_scheduler_execute(void)
{
    time_t now = time(NULL);

    // 如果是首次上电/电池耗尽冷启动，初始化下次执行时间为当前时间(立即执行)
    if (s_is_cold_boot) {
        LOG_INFO("Cold boot detected. Initializing DAQ schedule.");
        s_next_patrol_time = now;
        s_next_diagnosis_time = now;
        s_is_cold_boot = false;
    }

    // 检查配置，>0 代表任务启用
    bool patrol_enabled = (g_user_config.patrol > 0);
    bool diagnosis_enabled = (g_user_config.diagnosis > 0);

    bool run_patrol = patrol_enabled && (now >= s_next_patrol_time - WAKEUP_TOLERANCE_SEC);
    bool run_diagnosis = diagnosis_enabled && (now >= s_next_diagnosis_time - WAKEUP_TOLERANCE_SEC);

    // --- 任务重叠时的优先级策略：诊断优先，覆盖并跳过巡检 ---
    if (run_diagnosis && run_patrol) {
        LOG_INFO("Task overlap detected. Skipping Patrol in favor of Diagnosis.");
        run_patrol = false; // 取消巡检标记
    }

    esp_err_t err = ESP_OK;

    // --- 执行 DAQ 任务调度 ---
    if (run_diagnosis) {
        LOG_INFO("Executing Diagnosis task...");
        daq_worker_param_t param = {
            .rpm = g_user_config.rpm,
            .task_mode = TASK_MODE_DIAGNOSIS
        };
        err = start_daq_worker(&param);
        
        // 更新下次诊断时间
        s_next_diagnosis_time = now + (time_t)(g_user_config.diagnosis * 60);
        
        // 如果因为合并策略跳过了巡检，必须同步将巡检下一次时间后移，防止它在下次唤醒时立刻抢跑
        if (now >= s_next_patrol_time - WAKEUP_TOLERANCE_SEC) {
            s_next_patrol_time = now + (time_t)(g_user_config.patrol * 60);
        }
    } 
    else if (run_patrol) {
        LOG_INFO("Executing Patrol task...");
        daq_worker_param_t param = {
            .rpm = g_user_config.rpm,
            .task_mode = TASK_MODE_PATROLING
        };
        err = start_daq_worker(&param);
        
        // 更新下次巡检时间
        s_next_patrol_time = now + (time_t)(g_user_config.patrol * 60);
    } else {
        LOG_INFO("Woke up but no DAQ task scheduled to run right now.");
    }

    return err;
}

/**
 * @brief 获取下一次需要唤醒的时间差（微秒），用于喂给 esp_sleep_enable_timer_wakeup
 */
uint64_t daq_scheduler_get_sleep_time_us(void)
{
    time_t now = time(NULL);
    time_t next_wake = 0;

    bool patrol_enabled = (g_user_config.patrol > 0);
    bool diagnosis_enabled = (g_user_config.diagnosis > 0);

    if (!patrol_enabled && !diagnosis_enabled) {
        // 所有任务均被禁用，返回 0 代表不需要定时唤醒
        return 0; 
    } else if (patrol_enabled && diagnosis_enabled) {
        next_wake = (s_next_patrol_time < s_next_diagnosis_time) ? s_next_patrol_time : s_next_diagnosis_time;
    } else if (patrol_enabled) {
        next_wake = s_next_patrol_time;
    } else {
        next_wake = s_next_diagnosis_time;
    }

    // 防止因为逻辑误差导致过去的时间，做下边界保护 (1毫秒后重新评估)
    if (next_wake <= now) {
        return 1000ULL; 
    }

    uint64_t sleep_us = (uint64_t)(next_wake - now) * 1000000ULL;
    return sleep_us;
}

// ====================================================================
// 以下为兼容旧 API 的桩函数
// 由于 DAQ 重构为同步阻塞调度器（daq_scheduler_execute），不再有后台
// 周期性任务，因此 is_idle 恒为 true，pause/resume 和 start_task_daq 均为空操作。
// ====================================================================

esp_err_t start_task_daq(void)
{
    return ESP_OK;
}

bool task_daq_is_idle(void)
{
    return true;
}

esp_err_t task_daq_pause_periodic(void)
{
    return ESP_OK;
}

esp_err_t task_daq_resume_periodic(bool trigger_patrol_now)
{
    (void)trigger_patrol_now;
    return ESP_OK;
}

bool task_daq_periodic_enabled(void)
{
    return false;
}

esp_err_t task_daq_trigger_patrol_now(void)
{
    return ESP_OK;
}
