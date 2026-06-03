#include "task_mqtt_message.h"
#include "cJSON.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "config_manager.h"
#include "esp_system.h"
#include "mqtt_proxy.h" // 引入用于重启前关停网络的接口
#include "task_ota.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SN
#define SN 0
#endif

// 声明 http_proxy_get 接口 (将在之后的 http_proxy.h 中定义)
extern esp_err_t http_proxy_get(const char *url, char **out_response);

static void execute_config_update_sync(const char *task_id)
{
    char url[256];
    char *json_response = NULL;

    // 1. 拼接获取配置的完整 URL
    snprintf(url, sizeof(url), "http://%s/api/v1/sensor/config/%s", g_user_config.host, task_id);
    LOG_INFOF("Fetching config from: %s", url);

    // 2. 通过统一代理接口获取纯 JSON 字符串（自动抹平 4G AT 与 WiFi 差异）
    if (http_proxy_get(url, &json_response) == ESP_OK && json_response != NULL) {
        LOG_INFO("Config downloaded successfully, saving to SPIFFS...");
        
        // 3. 直接存盘覆盖 user_config.json
        esp_err_t err = config_manager_save_user_json(json_response);
        if (err == ESP_OK) {
            LOG_INFO("New config saved. Restarting system to apply changes...");
            free(json_response);
            
            // [极其重要] 同步架构下，由于我们准备原地重启系统，必须先手动优雅关闭 4G
            (void)mqtt_client_stop();
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();                   // 4. 重启单片机使新配置生效
        } else {
            LOG_ERROR("Failed to save new configuration");
        }
        free(json_response);
    } else {
        LOG_ERROR("Failed to fetch new configuration from server");
    }
}

void mqtt_message_process_pending_tasks(void)
{
    char url[256];
    char *json_response = NULL;

    // 1. 组装拉取任务列表的 URL
    snprintf(url, sizeof(url), "http://%s/api/v1/sensor/tasks/%u", g_user_config.host, (unsigned)SN);
    LOG_INFOF("Polling pending tasks from: %s", url);

    // 2. 通过 http_proxy 获取 JSON 响应
    if (http_proxy_get(url, &json_response) == ESP_OK && json_response != NULL) {
        cJSON *root = cJSON_Parse(json_response);
        if (cJSON_IsArray(root)) {
            int task_count = cJSON_GetArraySize(root);
            LOG_INFOF("Found %d pending tasks", task_count);
            
            cJSON *item = NULL;
            cJSON_ArrayForEach(item, root) {
                const cJSON *action_item = cJSON_GetObjectItemCaseSensitive(item, "action");
                const cJSON *task_id_item = cJSON_GetObjectItemCaseSensitive(item, "id");
                const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(item, "val");
                
                if (cJSON_IsNumber(action_item) && cJSON_IsString(task_id_item)) {
                    int action = action_item->valueint;
                    int val = cJSON_IsNumber(val_item) ? val_item->valueint : 0;
                    const char *task_id = task_id_item->valuestring;
                    
                    LOG_INFOF("Executing task synchronously: id=%s, action=%d, val=%d", task_id, action, val);

                    if (action == 1) {
                        execute_config_update_sync(task_id);
                    } else if (action == 2) {
                        LOG_INFO("Action 2 (OTA) received. Processing synchronously...");
                        execute_ota_update_sync(task_id);
                    } else if (action == 3) {
                        LOG_INFOF("Action 3 (Local Update) received. Applying val=%d", val);
                    }
                }
            }
        } else {
            LOG_INFO("No pending tasks found or invalid format.");
        }
        cJSON_Delete(root);
        free(json_response);
    } else {
        LOG_WARN("Failed to fetch pending tasks from server");
    }
}

// 兼容旧的接口签名 (空存根防止外部直接调用报错)
esp_err_t start_mqtt_message_task(void)
{
    return ESP_OK;
}

esp_err_t mqtt_message_task_submit(const char *topic, const uint8_t *data, size_t len)
{
    return ESP_OK;
}

void mqtt_message_task_register_action_handler(mqtt_action_handler_t handler)
{
}
