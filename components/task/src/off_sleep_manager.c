#include "off_sleep_manager.h"
#include "bsp_board.h"
#include "driver/gpio.h"

#include "drv_4g.h"
#include "drv_iis3dwb.h"
#include "drv_lis2dh12.h"
#include "logger.h"

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
  // 拔掉电源后，立即把相关数据线拉低到0V并锁定，防止寄生电流倒灌回芯片
  isolate_iis3dwb_pins();

  ret = gpio_set_level(BOARD_GPIO_SENSOR_EN, 1);
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to disable IIS3DWB power rail before OFF sleep: %s",
               esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

static esp_err_t off_sleep_manager_enter_wom_sleep(void) {

  esp_err_t ret = ESP_OK;

  (void)shutdown_4g_network();

  ret = off_sleep_prepare_capture_path();
  if (ret != ESP_OK) {
    goto rollback;
  }

  // 进入轻度睡眠等待唤醒
  ret = wom_lis2dh12_enter_light_sleep_until_wakeup();

  // 唤醒后，第一时间解除 IIS3DWB 引脚锁定，为后续重新上电做准备
  deisolate_iis3dwb_pins();

  if (ret != ESP_OK) {
    LOG_ERRORF("WoM light sleep failed: %s", esp_err_to_name(ret));
    (void)wom_lis2dh12_disable();
    goto rollback;
  }

  (void)wom_lis2dh12_disable();

  return ESP_OK;

rollback:
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
