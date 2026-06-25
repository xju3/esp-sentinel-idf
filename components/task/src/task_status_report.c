#include "task_status_report.h"
#include "bsp_4g.h"
#include "board_config.h"
#include "config_manager.h"
#include "logger.h"
#include "cJSON.h"

#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_http_client.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>

#ifndef REPORT_SN
#define REPORT_SN "UNKNOWN"
#endif

void task_status_report_execute(const char *task_id)
{
    LOG_INFOF("Executing Device Status Report Task: %s", task_id ? task_id : "NULL");

    float temperature = 0.0f;
    int rssi = -99;
    float voltage = 0.0f;
    uint64_t ts = (uint64_t)time(NULL);

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
    if (g_user_config.network == 1) {
        // Only valid if using 4G module
        if (bsp_4g_get_rssi(&rssi) != ESP_OK) {
            LOG_WARN("Failed to get 4G RSSI");
        }
    } else {
        LOG_INFO("Network is not 4G, skipping RSSI retrieval.");
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
    cJSON_AddStringToObject(root, "sn", REPORT_SN);
    cJSON_AddStringToObject(root, "task_id", task_id ? task_id : "");
    cJSON_AddNumberToObject(root, "ts", (double)ts);

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

    if (g_user_config.network == 1) {
        char *response = NULL;
        esp_err_t err = bsp_4g_http_post_json(url, json_str, &response);
        if (err == ESP_OK) {
            LOG_INFO("Successfully posted device status via 4G");
            if (response) free(response);
        } else {
            LOG_ERROR("Failed to post device status via 4G");
        }
    } else {
        esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client) {
            esp_http_client_set_header(client, "Content-Type", "application/json");
            esp_http_client_set_post_field(client, json_str, strlen(json_str));
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                LOG_INFOF("Successfully posted device status via WiFi, HTTP status = %d", esp_http_client_get_status_code(client));
            } else {
                LOG_ERRORF("Failed to post device status via WiFi: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(client);
        }
    }

    free(json_str);
}
