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
#include "bsp_wifi.h"
#include "fs_utils.h"
#include "logger.h"
#include "startup_gate.h"
#include "web_server.h"

static void restart_task(void *arg)
{
    LOG_INFO("restarting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    vTaskDelete(NULL);
}

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
    LOG_DEBUG("load user configuration.");
    user_config_t cfg = {0};
    if (config_manager_load(&cfg) != ESP_OK)
    {
        LOG_ERROR("config_manager_load failed.");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "load failed");
    }

    char *json = config_manager_create_json(&cfg);
    if (!json)
    {
        LOG_ERROR("Failed to build merged config json");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json build failed");
    }

    esp_err_t ret = send_json_string(req, json);
    free(json);
    return ret;
}

esp_err_t api_save_config_handler(httpd_req_t *req)
{
    int total = req->content_len;
    LOG_DEBUGF("saving user configurations, %d bytes", total);
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

    if (startup_gate_is_waiting_for_config())
    {
        startup_gate_mark_config_completed();
    }
    else
    {
        LOG_INFO("Config saved after boot gate; restarting to apply changes.");
        xTaskCreate(restart_task, "restart_task", 4096, NULL, 5, NULL);
    }
    return ESP_OK;
}

esp_err_t api_wifi_scan_start_handler(httpd_req_t *req)
{
    // 重置扫描完成标志，开始新的扫描
    s_scan_done = false;
    
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
    bool success = true;
    // 检查扫描是否完成
    if (!s_scan_done)
    {
        success = false;
        LOG_DEBUG("still in scan process.");
    }

    uint16_t ap_num = 0;
    esp_err_t err = esp_wifi_scan_get_ap_num(&ap_num);
    if (err != ESP_OK)
    {
        success = false;
        LOG_ERROR("get ap num failed");
    }

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)calloc(ap_num ? ap_num : 1, sizeof(wifi_ap_record_t));
    if (!ap_records)
    {
        success = false;
        LOG_ERROR("no mem");
    }

    err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
    if (err != ESP_OK)
    {
        success = false;
        free(ap_records);
    }

    cJSON *networks = cJSON_CreateArray();
    if (!networks)
    {
        success = false;
        free(ap_records);
    }
    
    if (!success)
    {
        cJSON_Delete(networks);
        free(ap_records);
        return send_json_string(req, "{\"status\":\"processing\"}");
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
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "processing");
    }
    LOG_DEBUGF("%s", json);
    esp_err_t ret = send_json_string(req, json);
    free(json);
    
    // 成功获取结果后，清除扫描缓存，重置扫描完成标志
    esp_wifi_clear_ap_list();
    s_scan_done = false;
    
    return ret;
}

esp_err_t api_get_consumption_handler(httpd_req_t *req)
{
    LOG_DEBUG("loading power consumption configuration.");
    if (!fsu_is_storage_mounted())
    {
        LOG_ERROR("Storage not mounted");
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "storage not mounted");
    }

    // 检查文件是否存在
    if (!fsu_file_exists(FILE_PATH_HW_CONSUMPTION))
    {
        LOG_ERRORF("Consumption file not found: %s", FILE_PATH_HW_CONSUMPTION);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "consumption file not found");
    }
    
    // 读取文件内容
    size_t len = 0;
    char *json = fsu_read_file_alloc(FILE_PATH_HW_CONSUMPTION, &len);
    if (!json)
    {
        LOG_ERRORF("Failed to read consumption file: %s", FILE_PATH_HW_CONSUMPTION);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "read failed");
    }
    
    LOG_DEBUGF("Successfully loaded consumption config: %d bytes", len);
    
    // 发送JSON响应
    esp_err_t ret = send_json_string(req, json);
    free(json);
    return ret;
}
