#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cJSON.h"
#include "config_manager.h"
#include "drv_4g.h"
#include "logger.h"
#include "task_binding.h"
#include "bsp_board.h"
#include "driver/gpio.h"
#include "drv_iis3dwb.h"

#if LIS2
#include "wom_lis2dh12.h"
#endif

// Report task complete result (HTTP POST /complete/<result>)
static esp_err_t report_task_complete(const char *task_id, int result_code) {
  if (!task_id || task_id[0] == '\0') {
    return ESP_OK;
  }
  char url[256];
  snprintf(url, sizeof(url), "http://%s/sensors/%s/complete/%d",
           g_user_config.api_host, task_id, result_code);

  LOG_INFOF("Reporting task complete result %d to %s", result_code, url);

  char *response = NULL;
  esp_err_t err = bsp_4g_http_post_json(url, "", &response);
  if (err == ESP_OK) {
    LOG_INFO("4G HTTP POST complete result reported successfully.");
  } else {
    LOG_ERROR("4G HTTP POST complete result report failed.");
  }
  free(response);
  return err;
}

static void binding_sleep_prepare_and_sleep(uint64_t sleep_us)
{
    (void)shutdown_4g_network();
    (void)drv_iis3dwb_enter_standby();

    esp_err_t sensor_power_err = gpio_hold_dis(BOARD_GPIO_SENSOR_EN);
    if (sensor_power_err == ESP_OK) {
        sensor_power_err = gpio_set_direction(BOARD_GPIO_SENSOR_EN, GPIO_MODE_INPUT_OUTPUT);
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
    vTaskDelay(pdMS_TO_TICKS(500)); 

    if (sleep_us > 0) {
        esp_sleep_enable_timer_wakeup(sleep_us);
    }
    
    esp_deep_sleep_start();
}

void task_binding_check_and_sleep(void)
{
    LOG_INFO("Device is not bound to any monitored device. Checking binding status via 4G...");
    bool is_bound = false;
    if (init_4g_network(NULL) == ESP_OK) {
        char url[256];
        snprintf(url, sizeof(url), "http://%s/api/v1/sensors/binding/%s",
                 g_user_config.api_host, g_user_config.sn);
        char *response = NULL;
        if (bsp_4g_http_get(url, &response) == ESP_OK && response != NULL) {
            cJSON *root = cJSON_Parse(response);
            if (root) {
                cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");
                if (cJSON_IsObject(data)) {
                    cJSON *dev_id_item = cJSON_GetObjectItemCaseSensitive(data, "device_id");
                    if (cJSON_IsString(dev_id_item) && dev_id_item->valuestring[0] != '\0') {
                        LOG_INFOF("Successfully retrieved binding info: device_id=%s", dev_id_item->valuestring);
                        config_manager_save_device_id(dev_id_item->valuestring);
                        is_bound = true;
                    }
                }
                cJSON_Delete(root);
            }
        } else {
            LOG_WARN("Failed to get binding status from server");
        }
        free(response);
    } else {
        LOG_WARN("Failed to initialize 4G network for binding check");
    }

    if (is_bound) {
        LOG_INFO("Device bound. Entering patrol sleep cycle without WoM.");
        uint64_t patrol_sleep_us = (uint64_t)g_user_config.patrol * 60ULL * 1000000ULL;
        binding_sleep_prepare_and_sleep(patrol_sleep_us);
    } else {
#ifdef DEV_MODE
#if DEV_MODE == 1
        uint64_t long_sleep_us = 1ULL * 60ULL * 1000000ULL; // 1 minute in dev mode
#else
        uint64_t long_sleep_us = 12ULL * 60ULL * 60ULL * 1000000ULL; // 12 hours
#endif
#else
        uint64_t long_sleep_us = 12ULL * 60ULL * 60ULL * 1000000ULL; // 12 hours
#endif
        LOG_INFOF("Device still not bound. Entering long sleep for %llu minutes...", long_sleep_us / (60ULL * 1000000ULL));
        binding_sleep_prepare_and_sleep(long_sleep_us);
    }
}

void task_binding_execute(const char *task_id) {
  LOG_INFOF("Executing Device Binding Update Task: %s",
            task_id ? task_id : "NULL");

  char url[256];
  snprintf(url, sizeof(url), "http://%s/api/v1/sensors/binding/%s",
           g_user_config.api_host, g_user_config.sn);

  char *response = NULL;
  esp_err_t err = bsp_4g_http_get(url, &response);
  if (err != ESP_OK || response == NULL) {
    LOG_ERROR("Failed to get binding status from server");
    free(response);
    return;
  }

  cJSON *root = cJSON_Parse(response);
  if (!root) {
    LOG_ERROR("Failed to parse binding JSON response");
    free(response);
    return;
  }

  cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");
  if (!cJSON_IsObject(data)) {
    LOG_ERROR("Invalid binding data format");
    cJSON_Delete(root);
    free(response);
    return;
  }

  const char *new_device_id = "";
  cJSON *dev_id_item = cJSON_GetObjectItemCaseSensitive(data, "device_id");
  if (cJSON_IsString(dev_id_item)) {
      new_device_id = dev_id_item->valuestring;
  } else {
      LOG_INFO("device_id is missing or not a string. Treating as completely unbound.");
  }
  bool needs_factory_reset = (new_device_id[0] == '\0');
  bool binding_changed =
      (strncmp(g_user_config.device_id, new_device_id, LEN_MAX_DEVICE_ID) != 0);

  // If completely unbound, restore factory settings
  if (needs_factory_reset) {
    LOG_INFO("Device completely unbound. Restoring factory settings...");
    report_task_complete(task_id, 1); // 1 means success

    // Give 4G module some time to finish transmitting the HTTP POST completely
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Restore factory settings: delete user_config.json
    if (remove(FILE_PATH_CONFIG_USER) == 0) {
      LOG_INFO("Successfully deleted user config file.");
    } else {
      LOG_WARN("Failed to delete user config file.");
    }

    LOG_INFO("Restarting system...");
    esp_restart();
  } else if (binding_changed) {
    LOG_INFOF("Binding changed from %s to %s", g_user_config.device_id,
              new_device_id);
    config_manager_save_device_id(new_device_id);
    report_task_complete(task_id, 1);
  } else {
    LOG_INFO("Binding relationship unchanged.");
    report_task_complete(task_id, 1);
  }

  cJSON_Delete(root);
  free(response);
}
