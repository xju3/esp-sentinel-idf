/**
 * @file daq_scheduler.c (由原 task_daq.c 重构而来)
 * @brief 基于深度睡眠和RTC的DAQ一次性调度器
 */
#include "task_daq.h"
#include "config_manager.h"
#include "logger.h"
#include "report_pipeline.h"
#include "server_report_task_scheduler.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_rtc_time.h"

// 容许提前唤醒的宽容度 (防止硬件定时器一点点抖动导致错过判断)
#define WAKEUP_TOLERANCE_US 2000000LL
#define IMMEDIATE_WAKEUP_US 1000000LL

// 使用 RTC 内存维护相对倒计时，彻底隔绝 NTP 服务器挂钟时间的跳变
RTC_DATA_ATTR static int64_t s_patrol_left_us = 0;
RTC_DATA_ATTR static uint64_t s_last_sleep_rtc_us = 0;

// 本次唤醒周期的RTC起点；每次启动都会由 daq_scheduler_execute() 设置。
static uint64_t s_cycle_start_rtc_us = 0;

// 本次运行中，哪些任务被触发执行了
static bool s_ran_patrol = false;
static bool s_is_wom_wakeup = false;

/**
 * @brief 从深度睡眠唤醒后调用的 DAQ 评估与执行入口
 * 此函数不依赖网络或绝对时间，只使用设备RTC相对计时
 */
esp_err_t daq_scheduler_execute(void)
{
    s_ran_patrol = false;

    const uint64_t rtc_now_us = esp_rtc_get_time_us();
    s_cycle_start_rtc_us = rtc_now_us;
    const bool woke_from_deep_sleep =
        (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED);

    if (!woke_from_deep_sleep || s_last_sleep_rtc_us == 0 ||
        rtc_now_us < s_last_sleep_rtc_us) {
        LOG_INFO("Cold boot detected. Initializing relative DAQ schedule.");
        s_patrol_left_us = 0;
        s_last_sleep_rtc_us = rtc_now_us;
    } else {
        // 使用RTC实际经过时间，而不是上次计划的睡眠时间。
        const int64_t slept_us = (int64_t)(rtc_now_us - s_last_sleep_rtc_us);
        s_patrol_left_us -= slept_us;
        server_report_task_on_wake(slept_us);
    }

    // 检查配置，>0 代表任务启用
    bool patrol_enabled = (g_user_config.patrol > 0);

    // 若禁用，将倒计时置为极大值防止误触发
    if (!patrol_enabled) s_patrol_left_us = INT64_MAX;

    if (patrol_enabled && s_patrol_left_us <= WAKEUP_TOLERANCE_US) {
        s_ran_patrol = true;
    }

    esp_err_t err = ESP_OK;

    bool task_executed = false;

    if (s_is_wom_wakeup) {
        LOG_INFO("Executing WoM wakeup report pipeline...");
        err = report_pipeline_run("0");
        s_is_wom_wakeup = false;
        task_executed = true;
    }

    if (s_ran_patrol) {
        server_report_task_clear("normal report due");
        LOG_INFO("Executing unified normal report pipeline...");
        err = report_pipeline_run(NULL);
        task_executed = true;
    }

    // Execute any server tasks that are due (either from previous deep sleep schedule, 
    // or newly received from the HTTP response of the report_pipeline_run above).
    while (server_report_task_is_due()) {
        char task_id[64] = {0};
        if (server_report_task_copy_due_id(task_id, sizeof(task_id))) {
            LOG_INFOF("Executing scheduled server report task: id=%s", task_id);
            err = report_pipeline_run(task_id);
            server_report_task_mark_attempted(task_id);
            task_executed = true;
        } else {
            break;
        }
    }

    if (!task_executed) {
        LOG_INFO("Woke up but no report task scheduled to run right now.");
    }

    return err;
}

/**
 * @brief 获取下一次需要唤醒的时间差（微秒），用于喂给 esp_sleep_enable_timer_wakeup
 */
uint64_t daq_scheduler_get_sleep_time_us(void)
{
    // 1. 只使用设备RTC计算本轮实际工作耗时，不依赖基站/NTP时间。
    const uint64_t rtc_now_us = esp_rtc_get_time_us();
    int64_t exec_time_us = 0;
    if (s_cycle_start_rtc_us > 0 && rtc_now_us >= s_cycle_start_rtc_us) {
        exec_time_us = (int64_t)(rtc_now_us - s_cycle_start_rtc_us);
    }

    // 保存即将进入深睡前的RTC读数；下次唤醒后用差值获得实际睡眠时长。
    s_last_sleep_rtc_us = rtc_now_us;

    bool patrol_enabled = (g_user_config.patrol > 0);

    int64_t p_period_us = (int64_t)(g_user_config.patrol * 60000000.0); // 分钟 -> 微秒

    // 2. 将之前剩余的倒计时，继续扣减掉本次的工作耗时
    s_patrol_left_us -= exec_time_us;
    server_report_task_after_work(exec_time_us);

    const bool normal_overdue_after_work =
        (!s_ran_patrol && patrol_enabled && s_patrol_left_us <= 0);
    if (normal_overdue_after_work) {
        server_report_task_clear("normal report overdue after server task");
        return (uint64_t)IMMEDIATE_WAKEUP_US;
    }

    // 3. 对刚执行过的任务满血复活，将透支的时间从下一个完整周期里扣除
    if (patrol_enabled && s_ran_patrol) {
        s_patrol_left_us += p_period_us;
    }

    // 4. 处理中途通过云端打开的任务，或云端动态缩短了周期的场景
    // 如果剩余时间变成了负数，或者剩余倒计时比当前最新配置的周期还要长，则强制对齐到新周期
    if (patrol_enabled && !s_ran_patrol && (s_patrol_left_us <= 0 || s_patrol_left_us > p_period_us)) {
        s_patrol_left_us = p_period_us;
    }

    // 5. 寻找下一次唤醒需要睡眠的最短倒计时
    int64_t next_sleep_us = -1;
    int64_t min_period_us = 0;

    if (patrol_enabled) {
        next_sleep_us = s_patrol_left_us;
        min_period_us = p_period_us;
    } else {
        next_sleep_us = -1;
        min_period_us = 0;
    }

    if (server_report_task_is_active()) {
        const int64_t server_left_us = server_report_task_left_us();
        if (server_left_us <= 0) {
            next_sleep_us = IMMEDIATE_WAKEUP_US;
        } else if (next_sleep_us < 0 || server_left_us < next_sleep_us) {
            next_sleep_us = server_left_us;
        }
    }

    if (next_sleep_us < 0) {
        return 0; // 全被禁用，且无服务器任务，无限休眠
    }

    // 6. 正常情况下，睡眠时间是周期剩余量：patrol周期 - 实际工作时间。
    // 若本轮工作已经耗尽或超过周期，则放弃已经错过的周期，重新锚定：
    // 完整睡眠一个patrol周期后再开始下一次采集，避免立即连续采集。
    if (next_sleep_us <= 0) {
        next_sleep_us = patrol_enabled ? p_period_us : IMMEDIATE_WAKEUP_US;
        if (patrol_enabled) s_patrol_left_us = p_period_us;
    }

    // 7. 防护：防止由于周期动态缩小等边缘配置跳变导致极大的睡眠时间
    if (min_period_us > 0 && next_sleep_us > min_period_us) {
        next_sleep_us = min_period_us;
    }

    return (uint64_t)next_sleep_us;
}

esp_err_t task_daq_trigger_patrol_now(void)
{
    s_patrol_left_us = 0;
    return ESP_OK;
}

esp_err_t task_daq_trigger_wom_patrol(void)
{
    s_is_wom_wakeup = true;
    return ESP_OK;
}
