#include "esp_attr.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include <string.h>
#include <time.h>

#include "bsp_wifi.h" // 引入 WiFi 接口
#include "config_manager.h"

#include "drv_4g.h" // 引入 4G 相关接口
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "esp_sntp.h" // 引入 WiFi 原生对时
#include "init.h"
#include "logger.h"

#include "system_lock.h"
#include "task_daq.h"

#include "task_ota.h"

#include "wom_lis2dh12.h" // 引入 WoM 接口

// 声明在 bsp_4g.c 中实现的 4G 对时函数
extern esp_err_t bsp_4g_sync_time(void);

// 定义事件组，用于主线程与后台网络任务的同步
static EventGroupHandle_t s_network_event_group = NULL;
#define NETWORK_DONE_BIT BIT0

// === 密集诊断模式状态 (存储在 RTC 内存，深睡掉电不丢失) ===
RTC_DATA_ATTR int g_dense_diag_remaining = 0; // 剩余密集诊断次数
RTC_DATA_ATTR int g_dense_diag_interval_s =
    300; // 密集诊断的时间间隔 (默认 300秒 = 5分钟)

// 供外部业务模块(如云端下发任务、或本地算法异常时)调用
void enable_dense_diagnostic(int times, int interval_seconds) {
  g_dense_diag_remaining = times;
  g_dense_diag_interval_s = interval_seconds;
}

static void network_bringup_task(void *pvParameters) {
  if (g_user_config.network == 1) {
    // LOG_INFO("Background: Connecting to 4G Network...");
    init_4g_network(NULL);
    // 异步更新一次基站时间，防止设备长期休眠带来的 RTC 晶振温漂
    (void)bsp_4g_sync_time();
  } else {
    // LOG_INFO("Background: Connecting to WiFi Network...");
    wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, NULL);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    // WiFi 模式下，非阻塞等待 SNTP 拿到时间
    time_t now = 0;
    int retries = 0;
    while (time(&now) < 1600000000 && retries < 10) {
      vTaskDelay(pdMS_TO_TICKS(500));
      retries++;
    }
  }

  // 通知主线程：网络及对时已经准备就绪
  if (s_network_event_group != NULL) {
    xEventGroupSetBits(s_network_event_group, NETWORK_DONE_BIT);
  }
  vTaskDelete(NULL);
}

void app_main(void) {
  // 1. 初始化基础外设与配置
  init_nvs();
  init_system_lock();
  esp_err_t cfg_err = config_manager_load(&g_user_config);
  if (cfg_err != ESP_OK) {
    LOG_ERRORF("Config load failed or RPM unsupported: 0x%X", cfg_err);
  }

  deisolate_lis2dh12_pins();

  // 2. 启动本地服务
  ESP_ERROR_CHECK(start_local_services());

  // 设置时区为东八区
  setenv("TZ", "CST-8", 1);
  tzset();

  time_t now = 0;
  time(&now);
  bool is_hot_wakeup = (now > 1600000000);

  s_network_event_group = xEventGroupCreate();

  if (is_hot_wakeup) {
    // 热唤醒：RTC 时间有效，直接进入后台连网，主线程瞬间放行执行 DAQ
    // LOG_INFOF("Hot wakeup detected. Valid RTC time: %ld. Starting network in
    // background...", (long)now);
    xTaskCreate(network_bringup_task, "net_bringup", 4096, NULL, 5, NULL);
  } else {
    // 冷启动：时间无效(断电重启)，必须强阻塞等待对时完成
    // LOG_INFO("Cold start. Initializing Network & Time Sync blockingly...");
    if (g_user_config.network == 1) {
      init_4g_network(NULL);
      for (int i = 0; i < 5; i++) {
        if (bsp_4g_sync_time() == ESP_OK)
          break;
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    } else {
      wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, NULL);
      esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
      esp_sntp_setservername(0, "pool.ntp.org");
      esp_sntp_init();
    }

    // LOG_INFO("Waiting for system time to be synchronized...");
    int retry_count = 0;
    while (time(&now) < 1600000000 && retry_count < 60) {
      vTaskDelay(pdMS_TO_TICKS(500));
      retry_count++;
    }
    if (now < 1600000000) {
      LOG_WARN("Time sync timeout! Scheduling will use un-synced time, which "
               "might cause errors.");
    } else {
      LOG_INFOF("Time synchronized successfully. Current time: %ld", (long)now);
    }
  }

  // 3. 执行单次 DAQ 调度决策
  // (判断当前时间是否需要采集，若需要则阻塞式采集并推入队列)
  // LOG_INFO("Evaluating DAQ schedule after wakeup...");
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    LOG_INFO("Wakeup caused by LIS2DH12 WoM! Forcing immediate patrol.");
    task_daq_trigger_wom_patrol();
  }
  daq_scheduler_execute();

  // LOG_INFO("Report pipeline finished.");

  // 5. 拉取并处理云端下发的同步任务 (OTA / 配置更新)
  // LOG_INFO("Checking for pending cloud tasks (OTA/Config)...");
  check_and_report_ota_status();

  // --- 修复：给后台网络及对时任务留出存活窗口 ---
  if (is_hot_wakeup) {
    // LOG_INFO("Hot wakeup: Waiting for background network and time sync to
    // complete..."); 最长等待 5
    // 秒。如果网络和对时提前完成，主线程会立刻被唤醒并放行，不会死等
    xEventGroupWaitBits(s_network_event_group, NETWORK_DONE_BIT, pdFALSE,
                        pdFALSE, pdMS_TO_TICKS(5000));
  }

  // --- 修复3：休眠前必须显式关断外部高功耗模块 ---
  // LOG_INFO("Shutting down peripherals before deep sleep...");
  if (g_user_config.network == 1) {
    (void)shutdown_4g_network(); // 通知 4G 模块 AT+QPOWD=1 关机并释放串口
  }
  (void)drv_iis3dwb_enter_standby(); // 传感器待机
  vTaskDelay(pdMS_TO_TICKS(500));    // 给 4G 模块一点点关机信号处理时间

  // 7. 计算下一次唤醒时间并进入深度睡眠
  uint64_t sleep_time_us = daq_scheduler_get_sleep_time_us();

  // === 拦截并动态覆盖休眠时间 (密集诊断逻辑) ===
  if (g_dense_diag_remaining > 0) {
    uint64_t dense_sleep_us = (uint64_t)g_dense_diag_interval_s * 1000000ULL;
    // 只有当密集诊断的时间比原定周期更短时，才进行覆盖
    if (sleep_time_us == 0 || dense_sleep_us < sleep_time_us) {
      sleep_time_us = dense_sleep_us;
      // LOG_INFOF("Dense diagnostic mode active (%d times left). Overriding
      // sleep to %d seconds...", g_dense_diag_remaining,
      // g_dense_diag_interval_s);
    }

    // 扣减一次剩余次数
    g_dense_diag_remaining--;
  }

  if (sleep_time_us > 0) {
    LOG_INFOF("All tasks finished. Entering deep sleep for %llu ms...",
              sleep_time_us / 1000ULL);
    esp_sleep_enable_timer_wakeup(sleep_time_us);
  } else {
    LOG_INFO("No periodic tasks enabled. Entering infinite deep sleep...");
  }

  // 在进入深睡之前，挂载并启用 LIS2DH12 的外部中断唤醒
  wom_lis2dh12_enable_deep_sleep_wakeup();

  esp_deep_sleep_start();
}
