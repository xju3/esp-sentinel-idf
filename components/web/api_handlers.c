#include "api_handlers.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config_manager.h"
#include "network_manager.h"
#include "fs_utils.h"
#include "logger.h"

static esp_err_t send_json_string(httpd_req_t *req, const char *json)
{
    if (!json)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no data");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_get_config_handler(httpd_req_t *req)
{
    if (config_manager_init() != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "init failed");
    }

    const char *path = NULL;
    if (fsu_is_user_mounted() && fsu_file_exists(CONFIG_USER_PATH))
    {
        path = CONFIG_USER_PATH;
    }
    else if (fsu_is_storage_mounted() && fsu_file_exists(CONFIG_DEFAULT_PATH))
    {
        path = CONFIG_DEFAULT_PATH;
    }

    if (!path)
    {
        LOG_ERROR("No config file found");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no config");
    }

    size_t len = 0;
    char *json = fsu_read_file_alloc(path, &len);
    if (!json)
    {
        LOG_ERRORF("Failed to read %s", path);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "read failed");
    }

    esp_err_t ret = send_json_string(req, json);
    free(json);
    return ret;
}

static void restart_task(void *arg)
{
    LOG_INFO("restarting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    vTaskDelete(NULL);
}

esp_err_t api_save_config_handler(httpd_req_t *req)
{
    int total = req->content_len;
    LOG_DEBUGF("saving user configurations, %s bytes", total);
    if (total <= 0 || total > 8192)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid length");
    }

    char *buf = (char *)calloc((size_t)total + 1, 1);
    if (!buf)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    }

    int received = 0;
    while (received < total)
    {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0)
        {
            free(buf);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error");
        }
        received += r;
    }

    esp_err_t err = config_manager_save_user_json(buf);
    free(buf);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Save user config failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
    }
    LOG_DEBUG("user configurations saved.");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"success\"}", HTTPD_RESP_USE_STRLEN);

    xTaskCreate(restart_task, "restart_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}


esp_err_t api_wifi_scan_start_handler(httpd_req_t *req)
{
    esp_err_t err = scan_wifi();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE)
    {
        LOG_ERRORF("WiFi scan start failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scan start failed");
    }

    return send_json_string(req, "{\"status\":\"processing\"}");
}

esp_err_t api_wifi_list_handler(httpd_req_t *req)
{
    // 检查扫描是否完成
    if (!s_scan_done)
    {
        return send_json_string(req, "{\"status\":\"processing\"}");
    }

    uint16_t ap_num = 0;
    esp_err_t err = esp_wifi_scan_get_ap_num(&ap_num);
    if (err != ESP_OK)
    {
        return send_json_string(req, "{\"status\":\"processing\"}");
    }

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)calloc(ap_num ? ap_num : 1, sizeof(wifi_ap_record_t));
    if (!ap_records)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    }

    err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
    if (err != ESP_OK)
    {
        free(ap_records);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scan list failed");
    }

    cJSON *networks = cJSON_CreateArray();
    if (!networks)
    {
        free(ap_records);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    }

    for (uint16_t i = 0; i < ap_num; ++i)
    {
        cJSON *obj = cJSON_CreateObject();
        cJSON_AddStringToObject(obj, "ssid", (const char *)ap_records[i].ssid);
        cJSON_AddNumberToObject(obj, "rssi", ap_records[i].rssi);
        cJSON_AddNumberToObject(obj, "enc", ap_records[i].authmode == WIFI_AUTH_OPEN ? 0 : 1);
        cJSON_AddItemToArray(networks, obj);
    }

    // 返回对象格式而不是直接数组，便于前端处理
    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        cJSON_Delete(networks);
        free(ap_records);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
    }
    cJSON_AddItemToObject(root, "networks", networks);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(ap_records);

    if (!json)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "encode failed");
    }

    LOG_DEBUG(json);
    esp_err_t ret = send_json_string(req, json);
    free(json);
    return ret;
}
