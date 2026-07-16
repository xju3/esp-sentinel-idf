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
#include "report_pipeline.h"
#include "task_daq.h"

#include "task_ota.h"
#include "esp_ota_ops.h"

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
  gpio_hold_dis(BOARD_GPIO_SENSOR_EN);
  // deisolate_iis3dwb_pins();
#if LIS2
  deisolate_lis2dh12_pins();
#endif
  deisolate_ds18b20_pin();
}

static esp_err_t power_off_sensors_for_deep_sleep(void) {
  esp_err_t ret = drv_iis3dwb_enter_standby();
  if (ret != ESP_OK) {
    LOG_WARNF("Failed to place IIS3DWB in standby: %s", esp_err_to_name(ret));
  }

  esp_err_t isolate_ret = isolate_ds18b20_pin();
  if (isolate_ret != ESP_OK) {
    LOG_WARNF("Failed to isolate DS18B20 pin: %s",
              esp_err_to_name(isolate_ret));
    if (ret == ESP_OK) {
      ret = isolate_ret;
    }
  }

  esp_err_t power_ret = gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  if (power_ret == ESP_OK) {
    power_ret = gpio_hold_en(BOARD_GPIO_SENSOR_EN);
  }
  if (power_ret != ESP_OK) {
    LOG_ERRORF("Failed to hold sensor power rail off for deep sleep: %s",
               esp_err_to_name(power_ret));
    if (ret == ESP_OK) {
      ret = power_ret;
    }
  }

  // GPIO2 is active-low sensor power enable. Without a deep-sleep hold it can
  // become high impedance after esp_deep_sleep_start() and turn the complete
  // sensor rail back on.
  gpio_deep_sleep_hold_en();
  LOG_INFOF("Sensor deep-sleep power state: SENSOR_EN=%d (off=1)",
            gpio_get_level(BOARD_GPIO_SENSOR_EN));
  return ret;
}
//
void app_main(void) {
  esp_err_t err = ESP_OK;
  bool has_report_work = false;

  const esp_app_desc_t *app_desc = esp_ota_get_app_description();
  LOG_INFOF("========================================");
  LOG_INFOF("Firmware Version: %s", app_desc->version);
  LOG_INFOF("========================================");

  // 1. 初始化基础外设与配置
  init_nvs();
  init_system_lock();
  deisolate_gpio_pins();

  // 先启动 DS18B20 转换；后续本地传感器初始化、配置加载和缓冲区准备
  // 都与转换等待重叠，正式 IIS3DWB 采样仍在温度结果读取之后开始。
  ESP_ERROR_CHECK(start_local_services());

  esp_err_t cfg_err = config_manager_load(&g_user_config);
  if (cfg_err != ESP_OK) {
    LOG_ERRORF("Config load failed or RPM unsupported: 0x%X", cfg_err);
  }

  // OTA镜像的本地确认不能依赖服务器。先取消回滚并持久化结果，
  // 完成通知等本轮正常检测报告上传成功后再补报。
  esp_err_t ota_finalize_err = task_ota_finalize_boot_status();
  if (ota_finalize_err != ESP_OK) {
    LOG_WARNF("OTA boot status finalization failed: %s",
              esp_err_to_name(ota_finalize_err));
  }

  // 2. 统一评估本次启动需要完成的工作，不按启动来源拆分业务流程。
#if LIS2
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    LOG_INFO("Wakeup caused by LIS2DH12 WoM! Forcing immediate patrol.");
    task_daq_trigger_wom_patrol();
  }
#endif

  err = daq_scheduler_prepare(&has_report_work);
  if (err != ESP_OK) {
    LOG_WARNF("DAQ schedule evaluation failed: %s", esp_err_to_name(err));
    goto sleep_prepare;
  }

  if (cfg_err != ESP_OK || g_user_config.sn[0] == '\0') {
    LOG_ERROR("Device configuration or SN is unavailable. Skipping acquisition and upload.");
    goto sleep_prepare;
  }

  if (!has_report_work) {
    goto sleep_prepare;
  }

  // 3. 先完成原始采样，再让4G启动与报告计算并行；上传前等待两者完成。
  //    这样既避免射频和电源纹波污染采样，又缩短整轮工作时间。
  err = daq_scheduler_execute(prepare_4g_network, NULL);
  if (err != ESP_OK) {
    LOG_WARNF("Capture or 4G preparation failed: %s", esp_err_to_name(err));
    goto sleep_prepare;
  }

  // 4. 正常检测报告已成功，服务器可用；此时再补报OTA完成结果。
  //    补报失败只保留NVS标记，不改变本轮正常业务结果。
  task_ota_report_pending_completion();

  // 历史报告只是附属补报环节，放在本轮必要业务完成之后；
  // 其成功或失败都不能改变本轮业务结果。
  (void)report_pipeline_flush_cache();

sleep_prepare:
  // 5. 统一释放本轮外部资源。
  (void)shutdown_4g_network(); // 通知4G模块关机并释放串口
  (void)power_off_sensors_for_deep_sleep();
  vTaskDelay(pdMS_TO_TICKS(500)); // 给 4G 模块一点点关机信号处理时间

  // 6. 计算下一次唤醒时间并进入深度睡眠
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
