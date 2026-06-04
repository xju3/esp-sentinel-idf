#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <string.h>
#include <time.h>

#include "init.h"
#include "config_manager.h"
#include "logger.h"
#include "web_server.h"
#include "machine_state.h"
#include "startup_gate.h"
#include "task_daq.h"
#include "task_fft.h"
#include "task_mqtt_message.h"
#include "task_rms.h"
#include "task_envelope.h"
#include "task_kurtosis.h"
#include "mqtt_proxy.h"
#include "drv_iis3dwb.h"
#include "bsp_4g.h"        // 引入 4G 相关接口
#include "bsp_wifi.h"      // 引入 WiFi 接口
#include "esp_sntp.h"      // 引入 WiFi 原生对时

// 声明在 bsp_4g.c 中实现的 4G 对时函数
extern esp_err_t bsp_4g_sync_time(void);

void app_main(void)
{
    // 1. 初始化基础外设与配置
    init_nvs();
    init_machine_state();
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));

    // 2. 启动本地服务
    ESP_ERROR_CHECK(start_local_services());

    // --- 修复1：先启动网络，再获取系统时间 ---
    LOG_INFO("Initializing Network & Time Sync...");
    
    // 设置时区为东八区
    setenv("TZ", "CST-8", 1);
    tzset();

    if (g_user_config.network == 1) {
        // 【4G 模式】
        LOG_INFO("Connecting to 4G Network...");
        init_4g_mqtt(NULL); // 内部会阻塞直到驻网成功并连接MQTT
        
        // 尝试从 4G 模组提取蜂窝基站时间，最多重试 5 次
        for (int i = 0; i < 5; i++) {
            if (bsp_4g_sync_time() == ESP_OK) break;
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    } else {
        // 【WiFi 模式】
        LOG_INFO("Connecting to WiFi Network...");
        wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, NULL);
        
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();
    }

    LOG_INFO("Waiting for system time to be synchronized...");
    time_t now = 0;
    int retry_count = 0;
    // 1600000000 约等于 2020年，如果时间小于此值说明未对时
    while (time(&now) < 1600000000 && retry_count < 60) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry_count++;
    }
    if (now < 1600000000) {
        LOG_WARN("Time sync timeout! Scheduling will use un-synced time, which might cause errors.");
    } else {
        LOG_INFOF("Time synchronized successfully. Current time: %ld", (long)now);
    }

    // 3. 执行单次 DAQ 调度决策 (判断当前时间是否需要采集，若需要则阻塞式采集并推入队列)
    LOG_INFO("Evaluating DAQ schedule after wakeup...");
    daq_scheduler_execute();

    // --- 修复2：严谨的全流水线排空等待 ---
    LOG_INFO("Waiting for data pipeline to drain...");
    bool pipeline_idle = false;
    while (!pipeline_idle) {
        pipeline_idle = true;
        // 检查所有已知队列是否有积压
        if (g_rms_job_queue && uxQueueMessagesWaiting(g_rms_job_queue) > 0) pipeline_idle = false;
        if (g_fft_job_queue && uxQueueMessagesWaiting(g_fft_job_queue) > 0) pipeline_idle = false;
        if (g_envelope_job_queue && uxQueueMessagesWaiting(g_envelope_job_queue) > 0) pipeline_idle = false;
        if (g_kurtosis_job_queue && uxQueueMessagesWaiting(g_kurtosis_job_queue) > 0) pipeline_idle = false;
        
        // 检查 FFT 任务的 busy 状态
        if (!task_fft_is_idle()) pipeline_idle = false;

        if (!pipeline_idle) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    // 额外留出 1.5 秒余量，确保各个分析任务把刚刚出队的最后一条数据也处理并上报完
    vTaskDelay(pdMS_TO_TICKS(1500));

    // 5. 拉取并处理云端下发的同步任务 (OTA / 配置更新 / 本地任务)
    LOG_INFO("Checking for pending cloud tasks (OTA/Config)...");
    mqtt_pending_tasks_result_t task_result = mqtt_message_process_pending_tasks();
    if (task_result == MQTT_PENDING_TASKS_NONE) {
        LOG_INFO("No pending cloud tasks.");
    }

    // --- 休眠前必须显式关断外部高功耗模块 ---
    LOG_INFO("Shutting down peripherals before deep sleep...");
    (void)mqtt_client_stop(); // 通知 4G 模块 AT+QPOWD=1 关机并释放串口
    (void)drv_iis3dwb_enter_standby(); // 传感器待机
    vTaskDelay(pdMS_TO_TICKS(500)); // 给 4G 模块一点点关机信号处理时间

    // 7. 计算下一次唤醒时间并进入深度睡眠
    uint64_t sleep_time_us = daq_scheduler_get_sleep_time_us();
    if (sleep_time_us > 0) {
        LOG_INFOF("All tasks finished. Entering deep sleep for %llu seconds...", sleep_time_us / 1000000ULL);
        esp_sleep_enable_timer_wakeup(sleep_time_us);
    } else {
        LOG_INFO("No periodic tasks enabled. Entering infinite deep sleep...");
    }

    esp_deep_sleep_start();
}
