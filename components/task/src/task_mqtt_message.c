#include "task_mqtt_message.h"
#include "cJSON.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "config_manager.h"
#include "mqtt_proxy.h" // 引入用于重启前关停网络的接口
#include "task_ota.h"
#include "http_proxy.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SN
#define SN 0
#endif

static mqtt_action_handler_t s_action_handler = NULL;

static esp_err_t execute_config_update_sync(const char *task_id)
{
    char url[256];
    char *json_response = NULL;
    esp_err_t ret = ESP_FAIL;

    // 1. 拼接获取配置的完整 URL
    snprintf(url, sizeof(url), "http://%s/api/v1/sensors/config/%s", g_user_config.host, task_id);
    // LOG_INFOF("Fetching config from: %s", url);

    // 2. 通过统一代理接口获取纯 JSON 字符串（自动抹平 4G AT 与 WiFi 差异）
    if (http_proxy_get(url, &json_response) == ESP_OK && json_response != NULL)
    {
        // LOG_INFO("Config downloaded successfully, saving to SPIFFS...");

        ret = config_manager_save_user_json(json_response);
        if (ret == ESP_OK)
        {
            // LOG_INFO("New config saved. It will be applied on the next wakeup.");
        }
        else
        {
            LOG_ERROR("Failed to save new configuration");
        }
        free(json_response);
    }
    else
    {
        LOG_ERROR("Failed to fetch new configuration from server");
    }

    return ret;
}

mqtt_pending_tasks_result_t mqtt_message_process_pending_tasks(void)
{
    char url[256];
    char *json_response = NULL;
    bool found_task = false;
    bool keep_4g_required = false;
    bool transport_shutdown = false;

    // 1. 组装拉取任务列表的 URL（去掉硬编码的 :3090 端口，由 g_user_config.host 统一管理）
    snprintf(url, sizeof(url), "http://%s/api/v1/sensors/tasks/%u", g_user_config.host, (unsigned)SN);
    // LOG_INFOF("Polling pending tasks from: %s", url);

    // 2. 通过 http_proxy 获取 JSON 响应
    if (http_proxy_get(url, &json_response) == ESP_OK && json_response != NULL)
    {
        cJSON *root = cJSON_Parse(json_response);
        if (cJSON_IsArray(root))
        {
            int task_count = cJSON_GetArraySize(root);
            LOG_INFOF("Found %d pending tasks", task_count);

            cJSON *item = NULL;
            cJSON_ArrayForEach(item, root)
            {
                const cJSON *action_item = cJSON_GetObjectItemCaseSensitive(item, "action");
                const cJSON *task_id_item = cJSON_GetObjectItemCaseSensitive(item, "id");
                const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(item, "val");

                if (cJSON_IsNumber(action_item) && cJSON_IsString(task_id_item))
                {
                    int action = action_item->valueint;
                    int val = cJSON_IsNumber(val_item) ? val_item->valueint : 0;
                    const char *task_id = task_id_item->valuestring;
                    found_task = true;
                    LOG_INFOF("Executing task synchronously: id=%s, action=%d, val=%d", task_id, action, val);

                    if (action < 10)
                    {
                        keep_4g_required = true;
                    }
                    else if (!transport_shutdown)
                    {
                        LOG_INFOF("Action %d does not require 4G. Shutting down 4G before local task execution.", action);
                        // shutdown 4g 的代码要改, 逻辑上不应该用 stop mqtt的方式进行.
                        (void)mqtt_client_stop();
                        transport_shutdown = true;
                    }

                    if (action == 1)
                    {
                        (void)execute_config_update_sync(task_id);
                    }
                    else if (action == 2)
                    {
                        LOG_DEBUG("Update firmware by OTA. Processing synchronously...");
                        execute_ota_update_sync(task_id);
                    }
                    else if (action == 3)
                    {
                        LOG_DEBUGF("发送设备状态至服务器, 包含电量, 4G信号强度, CPU温度, val=%d", val);
                    }
                    else if (action > 10 && action < 20)
                    {
                        LOG_DEBUGF("上传低频率检测FFT数据, 执续 action - 10次, 检测时间间隔为 val=%d 分钟.", val);
                    }
                    else if (action > 20 && action < 30)
                    {
                        LOG_DEBUGF("上传高频率检测FFT数据, 执续 action - 10次, 检测时间间隔为 val=%d 分钟.", val);
                    }
                    else if (s_action_handler != NULL)
                    {
                        (void)s_action_handler(action, task_id);
                    }
                    else
                    {
                        LOG_WARNF("No handler registered for action=%d", action);
                    }
                }
            }
        }
        else
        {
            LOG_DEBUG("No pending tasks found or invalid format.");
        }
        cJSON_Delete(root);
        free(json_response);
    }
    else
    {
        LOG_WARN("Failed to fetch pending tasks from server");
    }

    if (!found_task)
    {
        return MQTT_PENDING_TASKS_NONE;
    }
    return keep_4g_required ? MQTT_PENDING_TASKS_KEEP_4G : MQTT_PENDING_TASKS_CAN_SHUTDOWN_4G;
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
    s_action_handler = handler;
}
