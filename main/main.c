#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bsp_board.h"
#include "bsp_wifi.h" // 引入 WiFi 接口
#include "config_manager.h"

#include "drv_4g.h" // 引入 4G 相关接口
#include "drv_ds18b20.h"
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "init.h"
#include "logger.h"

#include "system_lock.h"
#include "task_daq.h"

#include "task_ota.h"

#include "wom_lis2dh12.h" // 引入 WoM 接口

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
  } else {
    // LOG_INFO("Background: Connecting to WiFi Network...");
    wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, NULL);
  }

  // 通知主线程：网络准备流程已经结束
  if (s_network_event_group != NULL) {
    xEventGroupSetBits(s_network_event_group, NETWORK_DONE_BIT);
  }
  vTaskDelete(NULL);
}

static void deisolate_gpio_pins() {
  gpio_deep_sleep_hold_dis();
  // deisolate_iis3dwb_pins();
#if LIS2
  deisolate_lis2dh12_pins();
#endif
  deisolate_ds18b20_pin();
}
//
void app_main(void) {
  // 1. 初始化基础外设与配置
  init_nvs();
  init_system_lock();
  esp_err_t cfg_err = config_manager_load(&g_user_config);
  if (cfg_err != ESP_OK) {
    LOG_ERRORF("Config load failed or RPM unsupported: 0x%X", cfg_err);
  }

  deisolate_gpio_pins();

  // 2. 启动本地服务
  ESP_ERROR_CHECK(start_local_services());

  // Cold/hot startup is a reset-source property, not a wall-clock property.
  // Wall-clock time is intentionally not synchronized or used by scheduling.
  const esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  const bool is_hot_wakeup = (wakeup_cause != ESP_SLEEP_WAKEUP_UNDEFINED);

  s_network_event_group = xEventGroupCreate();

  if (is_hot_wakeup) {
    // 深睡唤醒：后台连网，主线程直接执行基于相对时间的 DAQ 调度。
    xTaskCreate(network_bringup_task, "net_bringup", 4096, NULL, 5, NULL);
  } else {
    // 冷启动：只等待网络准备，不获取4G基站时间或NTP时间。
    if (g_user_config.network == 1) {
      init_4g_network(NULL);
    } else {
      wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass, NULL);
    }
  }

  // 3. 执行单次 DAQ 调度决策
  // (判断当前时间是否需要采集，若需要则阻塞式采集并推入队列)
  // LOG_INFO("Evaluating DAQ schedule after wakeup...");
#if LIS2
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    LOG_INFO("Wakeup caused by LIS2DH12 WoM! Forcing immediate patrol.");
    task_daq_trigger_wom_patrol();
  }
#endif
  daq_scheduler_execute();

  // LOG_INFO("Report pipeline finished.");

  // 5. 拉取并处理云端下发的同步任务 (OTA / 配置更新)
  // LOG_INFO("Checking for pending cloud tasks (OTA/Config)...");
  check_and_report_ota_status();

  // 给后台网络任务留出完成窗口。
  if (is_hot_wakeup) {
    xEventGroupWaitBits(s_network_event_group, NETWORK_DONE_BIT, pdFALSE,
                        pdFALSE, pdMS_TO_TICKS(90000));
  }

  // --- 修复3：休眠前必须显式关断外部高功耗模块 ---
  // LOG_INFO("Shutting down peripherals before deep sleep...");
  if (g_user_config.network == 1) {
    (void)shutdown_4g_network(); // 通知 4G 模块 AT+QPOWD=1 关机并释放串口
  }
  (void)drv_iis3dwb_enter_standby(); // 传感器待机
  gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  vTaskDelay(pdMS_TO_TICKS(500)); // 给 4G 模块一点点关机信号处理时间

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

#if LIS2
  // 在进入深睡之前，挂载并启用 LIS2DH12 的外部中断唤醒
  wom_lis2dh12_enable_deep_sleep_wakeup();
#endif

  esp_deep_sleep_start();
}
