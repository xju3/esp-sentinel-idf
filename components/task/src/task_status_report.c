#include "task_status_report.h"
#include "drv_4g.h"
#include "config_manager.h"
#include "logger.h"
#include "cJSON.h"

#include "driver/temperature_sensor.h"
#include <stdlib.h>

void task_status_report_execute(const char *task_id)
{
    LOG_INFOF("Executing Device Status Report Task: %s", task_id ? task_id : "NULL");

    float temperature = 0.0f;
    int rssi = -99;
    float voltage = 0.0f;

    // 1. Get MCU Temperature
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    if (temperature_sensor_install(&temp_sensor_config, &temp_sensor) == ESP_OK) {
        if (temperature_sensor_enable(temp_sensor) == ESP_OK) {
            temperature_sensor_get_celsius(temp_sensor, &temperature);
            temperature_sensor_disable(temp_sensor);
        }
        temperature_sensor_uninstall(temp_sensor);
    } else {
        LOG_WARN("Failed to install internal temperature sensor");
    }

    // 2. Get Battery Voltage
    // Due to hardware/software conflict on GPIO2, we cannot measure the battery voltage here.
    // Setting voltage to 0.0f to avoid reporting fake sensor power pin voltage.

    // 3. Get 4G RSSI
    if (bsp_4g_get_rssi(&rssi) != ESP_OK) {
        LOG_WARN("Failed to get 4G RSSI");
    }

    // 4. Build JSON Payload
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        LOG_ERROR("Failed to create JSON object");
        return;
    }

    cJSON_AddNumberToObject(root, "temperature", temperature);
    cJSON_AddNumberToObject(root, "rssi", rssi);
    cJSON_AddNumberToObject(root, "voltage", voltage);
    cJSON_AddStringToObject(root, "sensor_sn", g_user_config.sn);
    cJSON_AddStringToObject(root, "device_id", g_user_config.device_id);
    cJSON_AddStringToObject(root, "task_id", task_id ? task_id : "");

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_str) {
        LOG_ERROR("Failed to format JSON string");
        return;
    }

    // 5. Post to server
    char url[256];
    snprintf(url, sizeof(url), "http://%s/api/v1/sensors/status", g_user_config.api_host);
    
    LOG_INFOF("Posting device status to %s: %s", url, json_str);

    char *response = NULL;
    esp_err_t err = bsp_4g_http_post_json(url, json_str, &response);
    if (err == ESP_OK) {
        LOG_INFO("Successfully posted device status via 4G");
    } else {
        LOG_ERROR("Failed to post device status via 4G");
    }
    free(response);

    free(json_str);
}
