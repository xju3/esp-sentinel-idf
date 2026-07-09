#include "task_http_message.h"
#include "cJSON.h"
#include "logger.h"
#include "config_manager.h"
#include "drv_4g.h"
#include "task_ota.h"
#include "http_proxy.h"
#include "report_pipeline.h"
#include "server_report_task_scheduler.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "task_status_report.h"

#ifndef REPORT_SN
#define REPORT_SN "UNKNOWN"
#endif

static http_task_action_handler_t s_action_handler = NULL;

static int task_val_to_int(const cJSON *val_item)
{
    if (cJSON_IsNumber(val_item)) {
        return val_item->valueint;
    }
    if (cJSON_IsString(val_item) && val_item->valuestring) {
        return (int)strtol(val_item->valuestring, NULL, 10);
    }
    return 0;
}

static const char *task_val_to_string(const cJSON *val_item)
{
    if (cJSON_IsString(val_item)) {
        return val_item->valuestring;
    }
    return "";
}

static esp_err_t execute_config_update_sync(const char *task_id)
{
    char url[256];
    char *json_response = NULL;
    esp_err_t ret = ESP_FAIL;

    // 1. 拼接获取配置的完整 URL
    snprintf(url, sizeof(url), "http://%s/api/v1/sensors/config/%s", g_user_config.api_host, task_id);
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

http_pending_tasks_result_t http_message_process_task_array(const cJSON *task_array)
{
    if (!cJSON_IsArray(task_array)) {
        return HTTP_PENDING_TASKS_NONE;
    }

    bool found_task = false;
    bool keep_4g_required = false;
    bool transport_shutdown = false;

    int task_count = cJSON_GetArraySize(task_array);
    LOG_INFOF("Found %d pending tasks", task_count);

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, task_array)
    {
        const cJSON *action_item = cJSON_GetObjectItemCaseSensitive(item, "action");
        const cJSON *task_id_item = cJSON_GetObjectItemCaseSensitive(item, "id");
        const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(item, "val");

        if (cJSON_IsNumber(action_item) && cJSON_IsString(task_id_item))
        {
            int action = action_item->valueint;
            int val = task_val_to_int(val_item);
            const char *val_str = task_val_to_string(val_item);
            const char *task_id = task_id_item->valuestring;
            found_task = true;
            LOG_INFOF("Processing server task: id=%s, action=%d, val=%d", task_id, action, val);

            const bool is_repeated_report_action =
                (action >= 10 && action < 100) || (action >= 1000 && action < 3000);
            if (action < 10)
            {
                keep_4g_required = true;
            }
            else if (!transport_shutdown)
            {
                LOG_INFOF("Action %d does not require 4G. Shutting down 4G before local task execution.", action);
                if (g_user_config.network == 1)
                {
                    (void)shutdown_4g_network();
                }
                transport_shutdown = true;
            }

            if (action == 0)
            {
                LOG_DEBUG("Update firmware by OTA. Processing synchronously...");
                execute_ota_update_from_url_sync(task_id, val_str);
            }
            else if (action == 1)
            {
                (void)execute_config_update_sync(task_id);
            }
            else if (action == 2)
            {
                task_status_report_execute(task_id);
            }
            else if (is_repeated_report_action)
            {
                (void)server_report_task_schedule(task_id, action, val);
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

    if (!found_task) {
        return HTTP_PENDING_TASKS_NONE;
    }
    return keep_4g_required ? HTTP_PENDING_TASKS_KEEP_4G : HTTP_PENDING_TASKS_CAN_SHUTDOWN_4G;
}


esp_err_t start_http_message_task(void)
{
    return ESP_OK;
}

esp_err_t http_message_task_submit(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    return ESP_OK;
}

void http_message_task_register_action_handler(http_task_action_handler_t handler)
{
    s_action_handler = handler;
}
