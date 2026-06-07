/**
 * @file daq_scheduler.c (由原 task_daq.c 重构而来)
 * @brief 基于深度睡眠和RTC的DAQ一次性调度器
 */
#include "task_daq.h"
#include "config_manager.h"
#include "logger.h"
#include "report_pipeline.h"
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

    const bool run_patrol = patrol_enabled && (now >= s_next_patrol_time - WAKEUP_TOLERANCE_SEC);
    const bool run_diagnosis = diagnosis_enabled && (now >= s_next_diagnosis_time - WAKEUP_TOLERANCE_SEC);
    const bool run_report = run_patrol || run_diagnosis;

    esp_err_t err = ESP_OK;

    // 新机制不再区分 patrol / diagnosis：任一检测周期到期，都执行一次完整 normal report。
    if (run_report) {
        LOG_INFO("Executing unified normal report pipeline...");
        err = report_pipeline_run(NULL);

        if (run_patrol) {
            s_next_patrol_time = now + (time_t)(g_user_config.patrol * 60);
        }
        if (run_diagnosis) {
            s_next_diagnosis_time = now + (time_t)(g_user_config.diagnosis * 60);
        }
    } else {
        LOG_INFO("Woke up but no report task scheduled to run right now.");
    }

    // If one schedule was never initialized because the matching task is disabled,
    // keep it away from the past before it is re-enabled by a config update.
    if (diagnosis_enabled && s_next_diagnosis_time <= 0) {
        s_next_diagnosis_time = now + (time_t)(g_user_config.diagnosis * 60);
    }
    if (patrol_enabled && s_next_patrol_time <= 0) {
        s_next_patrol_time = now + (time_t)(g_user_config.patrol * 60);
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
