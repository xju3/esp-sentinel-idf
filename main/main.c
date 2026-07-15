#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bsp_board.h"
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

// === 密集诊断模式状态 (存储在 RTC 内存，深睡掉电不丢失) ===
RTC_DATA_ATTR int g_dense_diag_remaining = 0; // 剩余密集诊断次数
RTC_DATA_ATTR int g_dense_diag_interval_s =
    300; // 密集诊断的时间间隔 (默认 300秒 = 5分钟)

// 供外部业务模块(如云端下发任务、或本地算法异常时)调用
void enable_dense_diagnostic(int times, int interval_seconds) {
  g_dense_diag_remaining = times;
  g_dense_diag_interval_s = interval_seconds;
}

static esp_err_t prepare_4g_network(void *ctx) {
  (void)ctx;
  return init_4g_network(NULL);
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

  // 3. 统一评估本次启动需要完成的工作，不按启动来源拆分业务流程。
#if LIS2
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    LOG_INFO("Wakeup caused by LIS2DH12 WoM! Forcing immediate patrol.");
    task_daq_trigger_wom_patrol();
  }
#endif

  bool has_report_work = false;
  esp_err_t err = daq_scheduler_prepare(&has_report_work);
  if (err != ESP_OK) {
    LOG_WARNF("DAQ schedule evaluation failed: %s", esp_err_to_name(err));
    goto sleep_prepare;
  }

  const bool ota_status_pending = task_ota_status_pending();
  if (!has_report_work && !ota_status_pending) {
    goto sleep_prepare;
  }

  // 4. 先完成采集与计算，再启动4G，避免驻网期间的射频和电源纹波污染采样。
  if (has_report_work) {
    err = daq_scheduler_execute(prepare_4g_network, NULL);
  } else {
    err = prepare_4g_network(NULL);
  }
  if (err != ESP_OK) {
    LOG_WARNF("Capture or 4G preparation failed: %s", esp_err_to_name(err));
    goto sleep_prepare;
  }

  // 5. 网络确认可用后，补报OTA重启结果。服务器任务由报告响应处理。
  check_and_report_ota_status();

sleep_prepare:
  // 6. 统一释放本轮外部资源。
  (void)shutdown_4g_network(); // 通知4G模块关机并释放串口
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
