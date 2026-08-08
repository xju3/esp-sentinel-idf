#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "bsp_board.h"
#include "config_manager.h"

#include "cJSON.h"
#include "drv_4g.h" // 引入 4G 相关接口
#include "drv_ds18b20.h"
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "init.h"
#include "logger.h"

#include "report_pipeline.h"
#include "system_lock.h"
#include "task_daq.h"

#include "esp_ota_ops.h"
#include "esp_heap_caps.h"
#include "task_ota.h"
#include "task_binding.h"

#include "wom_lis2dh12.h" // 引入 WoM 接口

#include <stdio.h>
#include <string.h>

// === 密集诊断模式状态 (存储在 RTC 内存，深睡掉电不丢失) ===
RTC_DATA_ATTR int g_dense_diag_remaining = 0; // 剩余密集诊断次数
RTC_DATA_ATTR int g_dense_diag_interval_s =
    300; // 密集诊断的时间间隔 (默认 300秒 = 5分钟)

// 供外部业务模块(如云端下发任务、或本地算法异常时)调用
void enable_dense_diagnostic(int times, int interval_seconds) {
  g_dense_diag_remaining = times;
  g_dense_diag_interval_s = interval_seconds;
}

#ifndef CONFIG_SENTINEL_IIS3DWB_COMPARE_MODE

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

#endif

#ifdef CONFIG_SENTINEL_IIS3DWB_COMPARE_MODE

#define IIS3DWB_COMPARE_POINTS 16384U

static void free_compare_buffers(void) {
  heap_caps_free(g_user_config.vib_buf);
  heap_caps_free(g_user_config.fft_scratch);
  heap_caps_free(g_user_config.fft_mag);
  heap_caps_free(g_user_config.fft_work_buf);
  g_user_config.vib_buf = NULL;
  g_user_config.fft_scratch = NULL;
  g_user_config.fft_mag = NULL;
  g_user_config.fft_work_buf = NULL;
}

static esp_err_t prepare_compare_buffers(void) {
  memset(&g_user_config, 0, sizeof(g_user_config));
  snprintf(g_user_config.sn, sizeof(g_user_config.sn), "STEVAL-MKI208V1K");
  g_user_config.fft_points = IIS3DWB_COMPARE_POINTS;
  g_user_config.range_g = 2;

  g_user_config.vib_buf = heap_caps_calloc(
      IIS3DWB_COMPARE_POINTS * 3U, sizeof(float),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_user_config.fft_scratch = heap_caps_malloc(
      IIS3DWB_COMPARE_POINTS * sizeof(float),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_user_config.fft_mag = heap_caps_calloc(
      IIS3DWB_COMPARE_POINTS / 2U, sizeof(float),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_user_config.fft_work_buf = heap_caps_calloc(
      IIS3DWB_COMPARE_POINTS, sizeof(float),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!g_user_config.vib_buf || !g_user_config.fft_scratch ||
      !g_user_config.fft_mag || !g_user_config.fft_work_buf) {
    free_compare_buffers();
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void app_main(void) {
  const esp_app_desc_t *app_desc = esp_app_get_description();
  LOG_INFOF("Firmware Version: %s", app_desc->version);
  LOG_INFO("STEVAL-MKI208V1K comparison mode");
  LOG_INFOF("SPI pins: CS=%d SCL=%d SDA/MOSI=%d SDO/MISO=%d",
            IIS3DWB_PIN_NUM_CS, IIS3DWB_PIN_NUM_SCL,
            IIS3DWB_PIN_NUM_SDA, IIS3DWB_PIN_NUM_SDO);
  LOG_INFOF("Capture settings: fs=26667 Hz, points=%u, requested_range=2g",
            IIS3DWB_COMPARE_POINTS);

  esp_err_t err = prepare_compare_buffers();
  if (err == ESP_OK) {
    report_payload_t *payload = NULL;
    err = report_pipeline_capture_vibration_only(NULL, &payload);
    if (err == ESP_OK) {
      const char *json = report_pipeline_payload_json(payload);
      printf("\nIIS3DWB_COMPARE_JSON_BEGIN\n%s\nIIS3DWB_COMPARE_JSON_END\n",
             json ? json : "");
      fflush(stdout);
    }
    report_pipeline_discard(payload);
    if (!heap_caps_check_integrity_all(true)) {
      LOG_ERROR("Heap integrity check failed after comparison capture");
      err = ESP_FAIL;
    } else {
      LOG_INFO("Heap integrity check passed after comparison capture");
    }
  }

  if (err != ESP_OK) {
    LOG_ERRORF("IIS3DWB comparison capture failed: %s",
               esp_err_to_name(err));
  } else {
    LOG_INFO("IIS3DWB comparison capture complete; reset to capture again");
  }
  // This is a one-shot comparison firmware. Keep the large acquisition
  // buffers alive while the board idles; freeing them here previously caused
  // the post-report reboot seen on ESP32-S3/PSRAM.

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#else

//
void app_main(void) {
  esp_err_t err = ESP_OK;
  bool has_report_work = false;

  const esp_app_desc_t *app_desc = esp_app_get_description();
  LOG_INFOF("========================================");
  LOG_INFOF("Firmware Version: %s", app_desc->version);
  LOG_INFOF("========================================");

  // 1. 初始化基础外设与配置
  init_nvs();
  init_system_lock();
  deisolate_gpio_pins();

  esp_err_t cfg_err = config_manager_load(&g_user_config);
  if (cfg_err != ESP_OK) {
    LOG_ERRORF("Config load failed: 0x%X", cfg_err);
  }

  // OTA镜像的本地确认不能依赖服务器。先取消回滚并持久化结果，
  // 完成通知等本轮正常检测报告上传成功后再补报。
  esp_err_t ota_finalize_err = task_ota_finalize_boot_status();
  if (ota_finalize_err != ESP_OK) {
    LOG_WARNF("OTA boot status finalization failed: %s",
              esp_err_to_name(ota_finalize_err));
  }

  // Binding is checked before DAQ scheduling. RPM is evaluated later by the
  // detection pipeline and never determines whether startup may continue.
  if (g_user_config.sn[0] == '\0') {
    LOG_ERROR("Device SN is unavailable. Skipping binding check and acquisition.");
    goto sleep_prepare;
  }
  if (g_user_config.device_id[0] == '\0') {
    task_binding_check_and_sleep();

    // A newly discovered binding has just been persisted. Reload it now so
    // this same boot obtains the current profile and detection buffers.
    cfg_err = config_manager_load(&g_user_config);
    if (cfg_err != ESP_OK || g_user_config.device_id[0] == '\0') {
      LOG_ERRORF("Failed to activate retrieved binding: 0x%X", cfg_err);
      goto sleep_prepare;
    }
  }
  if (cfg_err != ESP_OK) {
    LOG_ERROR("Runtime configuration is unavailable. Skipping acquisition and upload.");
    goto sleep_prepare;
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

  // 如果执行到这里，说明设备已经绑定，正式进入检测工作流程。
  // 先启动 DS18B20 转换；后续任务与网络加载可与转换等待重叠，
  // 正式 IIS3DWB 采样仍在温度结果读取之后开始。
  ESP_ERROR_CHECK(start_local_services());

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
  (void)shutdown_4g_network();       // 通知4G模块关机并释放串口
  (void)drv_iis3dwb_enter_standby(); // 传感器待机
  esp_err_t sensor_power_err = gpio_hold_dis(BOARD_GPIO_SENSOR_EN);
  if (sensor_power_err == ESP_OK) {
    sensor_power_err =
        gpio_set_direction(BOARD_GPIO_SENSOR_EN, GPIO_MODE_INPUT_OUTPUT);
  }
  if (sensor_power_err == ESP_OK) {
    sensor_power_err = gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  }
  int sensor_power_level = gpio_get_level(BOARD_GPIO_SENSOR_EN);
  if (sensor_power_err != ESP_OK || sensor_power_level != 1) {
    LOG_ERRORF("Failed to switch sensor power off: err=%s SENSOR_EN=%d",
               esp_err_to_name(sensor_power_err), sensor_power_level);
  } else {
    LOG_INFO("Sensor power confirmed off: SENSOR_EN=1");
  }
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

#endif
