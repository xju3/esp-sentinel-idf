#include "off_sleep_manager.h"
#include "bsp_board.h"
#include "driver/gpio.h"

#include "bsp_4g.h"
#include "config_manager.h"
#include "drv_iis3dwb.h"
#include "logger.h"
#include "machine_state.h"
#include "sdkconfig.h"
#include "task_daq.h"
#include "task_fft.h"
#include "task_http_message.h" // 引入同步拉取接口
#include "wom_lis2dh12.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OFF_SLEEP_TASK_STACK_SIZE 4096
#define OFF_SLEEP_TASK_PRIORITY 5
#define OFF_SLEEP_WAIT_STEP_MS 20U
#define OFF_SLEEP_FFT_IDLE_TIMEOUT_MS 5000U

static TaskHandle_t s_off_sleep_task = NULL;
static volatile bool s_sleep_requested = false;

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

  LOG_INFO("No pending cloud tasks logic needed here. Proceeding to sleep.");
  LOG_INFO("All tasks processed. Shutting down transport before OFF sleep...");
  if (g_user_config.network == 1) {
    (void)shutdown_4g_network();
  }

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

  gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY); // SCL - IO12
  gpio_set_pull_mode(GPIO_NUM_11, GPIO_PULLUP_ONLY); // MOSI - IO11
  gpio_set_pull_mode(GPIO_NUM_10, GPIO_PULLUP_ONLY); // MISO - IO10

  gpio_hold_en(GPIO_NUM_9);  // CS
  gpio_hold_en(GPIO_NUM_10); // MISO
  gpio_hold_en(GPIO_NUM_12); // SCLK
  gpio_hold_en(GPIO_NUM_11); // MOSI
  gpio_deep_sleep_hold_en();

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
