#include "off_sleep_manager.h"
#include "bsp_board.h"
#include "driver/gpio.h"

#include "bsp_4g.h"
#include "config_manager.h"
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "logger.h"
#include "machine_state.h"
#include "sdkconfig.h"
#include "task_daq.h"
#include "task_fft.h"
#include "task_http_message.h" // 引入同步拉取接口
#include "wom_lis2dh12.h"

#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OFF_SLEEP_TASK_STACK_SIZE 4096
#define OFF_SLEEP_TASK_PRIORITY 5
#define OFF_SLEEP_WAIT_STEP_MS 20U
#define OFF_SLEEP_FFT_IDLE_TIMEOUT_MS 5000U

static TaskHandle_t s_off_sleep_task = NULL;
static volatile bool s_sleep_requested = false;

static esp_err_t isolate_lis2dh12_pins(void) {
  // 3. 释放普通 GPIO 状态
  gpio_reset_pin(LIS2DH12_PIN_NUM_SDO); // R45: LIS2DH12TR_MISO
  gpio_reset_pin(LIS2DH12_PIN_NUM_SDA); // R46: LIS2DH12TR_MOSI
  gpio_reset_pin(LIS2DH12_PIN_NUM_SCL); // R47: LIS2DH12TR_SCL
  gpio_reset_pin(LIS2DH12_PIN_NUM_CS);  // R44: LIS2DH12TR_CS

  gpio_set_direction(LIS2DH12_PIN_NUM_SDO, GPIO_MODE_INPUT);
  gpio_set_direction(LIS2DH12_PIN_NUM_SDA, GPIO_MODE_INPUT);
  gpio_set_direction(LIS2DH12_PIN_NUM_SCL, GPIO_MODE_INPUT);
  gpio_set_direction(LIS2DH12_PIN_NUM_CS, GPIO_MODE_INPUT);

  gpio_pullup_dis(LIS2DH12_PIN_NUM_SDO);
  gpio_pulldown_dis(LIS2DH12_PIN_NUM_SDO);
  gpio_pullup_dis(LIS2DH12_PIN_NUM_SDA);
  gpio_pulldown_dis(LIS2DH12_PIN_NUM_SDA);
  gpio_pullup_dis(LIS2DH12_PIN_NUM_SCL);
  gpio_pulldown_dis(LIS2DH12_PIN_NUM_SCL);
  gpio_pullup_dis(LIS2DH12_PIN_NUM_CS);
  gpio_pulldown_dis(LIS2DH12_PIN_NUM_CS);

  // 4. 对 RTC GPIO 做隔离
  rtc_gpio_isolate(LIS2DH12_PIN_NUM_SDO); // R45
  rtc_gpio_isolate(LIS2DH12_PIN_NUM_SDA); // R46，重点
  rtc_gpio_isolate(LIS2DH12_PIN_NUM_SCL); // R47
  rtc_gpio_isolate(LIS2DH12_PIN_NUM_CS);  // R44，可选但建议测试

  return ESP_OK;
}

static esp_err_t off_sleep_prepare_capture_path(void) {
  esp_err_t ret = drv_iis3dwb_enter_standby();
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to place IIS3DWB into standby before OFF sleep: %s",
               esp_err_to_name(ret));
    return ret;
  }

  gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to disable IIS3DWB power rail before OFF sleep: %s",
               esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

static esp_err_t off_sleep_wait_for_system_idle(void) {
  const TickType_t wait_step = pdMS_TO_TICKS(OFF_SLEEP_WAIT_STEP_MS);
  const TickType_t timeout_ticks = pdMS_TO_TICKS(OFF_SLEEP_FFT_IDLE_TIMEOUT_MS);
  const TickType_t start_ticks = xTaskGetTickCount();

  while (!task_daq_is_idle()) {
    if ((xTaskGetTickCount() - start_ticks) >= timeout_ticks) {
      LOG_WARN("Timed out waiting for DAQ/FFT pipeline to become idle before "
               "OFF sleep");
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(wait_step);
  }

  return ESP_OK;
}

static esp_err_t off_sleep_manager_enter_wom_sleep(void) {
  esp_err_t ret = task_daq_pause_periodic();
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to pause periodic DAQ before OFF sleep: %s",
               esp_err_to_name(ret));
    return ret;
  }

  bool periodic_paused = true;

  ret = off_sleep_wait_for_system_idle();
  if (ret != ESP_OK) {
    goto rollback;
  }

  if (g_user_config.network == 1) {
    (void)shutdown_4g_network();
  }

  isolate_lis2dh12_pins();

  ret = off_sleep_prepare_capture_path();
  if (ret != ESP_OK) {
    goto rollback;
  }

  set_machine_state(STATE_OFF);

  ret = wom_lis2dh12_enter_light_sleep_until_wakeup();
  if (ret != ESP_OK) {
    LOG_ERRORF("WoM light sleep failed: %s", esp_err_to_name(ret));
    (void)wom_lis2dh12_disable();
    goto rollback;
  }

  (void)wom_lis2dh12_disable();
  set_machine_state(STATE_TRANSIENT);

  if (periodic_paused) {
    (void)task_daq_resume_periodic(true);
  }

  return ESP_OK;

rollback:
  set_machine_state(STATE_TRANSIENT);
  if (periodic_paused) {
    (void)task_daq_resume_periodic(false);
  }
  return ret;
}

static void off_sleep_task_entry(void *arg) {
  (void)arg;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!s_sleep_requested) {
      continue;
    }

    esp_err_t ret = off_sleep_manager_enter_wom_sleep();
    if (ret != ESP_OK) {
      LOG_WARNF("OFF sleep request failed: %s", esp_err_to_name(ret));
    }

    s_sleep_requested = false;
  }
}

esp_err_t start_off_sleep_manager(void) {
  if (s_off_sleep_task != NULL) {
    return ESP_OK;
  }

  if (xTaskCreate(off_sleep_task_entry, "off_sleep", OFF_SLEEP_TASK_STACK_SIZE,
                  NULL, OFF_SLEEP_TASK_PRIORITY, &s_off_sleep_task) != pdPASS) {
    s_off_sleep_task = NULL;
    LOG_ERROR("Failed to create OFF sleep manager task");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t off_sleep_manager_request_sleep(void) {
  if (s_off_sleep_task == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  s_sleep_requested = true;
  xTaskNotifyGive(s_off_sleep_task);
  return ESP_OK;
}

bool off_sleep_manager_sleep_requested(void) { return s_sleep_requested; }
