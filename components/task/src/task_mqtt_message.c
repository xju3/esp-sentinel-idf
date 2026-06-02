#include "task_mqtt_message.h"
#include "cJSON.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "config_manager.h"
#include "esp_system.h"
#include "machine_state.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SN
#define SN 0
#endif

#define MQTT_MESSAGE_QUEUE_LEN 5
#define MQTT_MESSAGE_TASK_STACK_SIZE 4096
#define MQTT_MESSAGE_MAX_JSON_LEN 1024

typedef struct {
    char *topic;
    char *json;
    size_t len;
} mqtt_message_item_t;

static QueueHandle_t s_mqtt_message_queue = NULL;
static TaskHandle_t s_mqtt_message_task = NULL;
static mqtt_action_handler_t s_action_handler = NULL;

static void mqtt_message_free_item(mqtt_message_item_t *item)
{
    if (item == NULL) {
        return;
    }
    free(item->topic);
    free(item->json);
    item->topic = NULL;
    item->json = NULL;
    item->len = 0;
}

// 声明 http_proxy_get 接口 (将在之后的 http_proxy.h 中定义)
extern esp_err_t http_proxy_get(const char *url, char **out_response);

static void task_fetch_config(void *arg)
{
    char *task_id = (char *)arg;
    char url[256];
    char *json_response = NULL;

    // 注意：为防止任务调度空隙导致锁归零，此任务的唤醒锁已在其创建前被申请（锁的接力）

    // 1. 拼接获取配置的完整 URL
    snprintf(url, sizeof(url), "http://%s:8000/api/v1/sensors/task/%s", g_user_config.host, task_id);
    LOG_INFOF("Fetching config from: %s", url);

    // 2. 通过统一代理接口获取纯 JSON 字符串（自动抹平 4G AT 与 WiFi 差异）
    if (http_proxy_get(url, &json_response) == ESP_OK && json_response != NULL) {
        LOG_INFO("Config downloaded successfully, saving to SPIFFS...");
        
        // 3. 直接存盘覆盖 user_config.json
        esp_err_t err = config_manager_save_user_json(json_response);
        if (err == ESP_OK) {
            LOG_INFO("New config saved. Restarting system to apply changes...");
            vTaskDelay(pdMS_TO_TICKS(1000)); // 确保文件系统数据完全落盘
            esp_restart();                   // 4. 重启单片机使新配置生效
        } else {
            LOG_ERROR("Failed to save new configuration");
        }
        free(json_response);
    } else {
        LOG_ERROR("Failed to fetch new configuration from server");
    }

    system_wake_lock_release(); // 任务结束释放锁
    free(task_id);
    vTaskDelete(NULL);
}

static void mqtt_message_handle_action(int action, const char *task_id)
{
    // Action 1: 远端下发了新的配置更新指令
    if (action == 1) {
        if (task_id != NULL && strlen(task_id) > 0) {
            char *task_id_copy = strdup(task_id);
            if (task_id_copy) {
                // 接力锁：在创建后台长耗时任务前加锁，防止新任务还没被调度时主锁释放导致网络断开
                system_wake_lock_acquire();
                if (xTaskCreate(task_fetch_config, "fetch_cfg", 4096, task_id_copy, 4, NULL) != pdPASS) {
                    LOG_ERROR("Failed to create fetch_cfg task");
                    system_wake_lock_release();
                    free(task_id_copy);
                }
            }
        } else {
            LOG_WARN("Action 1 received but task_id is empty");
        }
        return;
    }

    // Action 2: 预留给 OTA 升级等长耗时网络任务
    if (action == 2) {
        LOG_INFO("Action 2 (OTA) received. (Placeholder)");
        return;
    }

    if (s_action_handler != NULL) {
        esp_err_t err = s_action_handler(action, task_id);
        if (err != ESP_OK) {
            LOG_WARNF("MQTT action handler failed: action=%d err=%s",
                      action,
                      esp_err_to_name(err));
        }
        return;
    }

    LOG_INFOF("MQTT action received: action=%d task_id=%s", action, task_id != NULL ? task_id : "");
}

static void mqtt_message_process_json(const char *topic, const char *json)
{
    if (!json) return;

    // 处理 4G URC 带来的外层引号: +QMTRECV: 0,0,"topic","{"task_id"...}"
    const char *parse_str = json;
    if (parse_str[0] == '"') {
        parse_str++; // 跳过第一个引号, cJSON_Parse 会在遇到末尾引号时自动安全停止
    }

    cJSON *root = cJSON_Parse(parse_str);
    if (!cJSON_IsObject(root)) {
        LOG_WARN("MQTT message ignored: invalid JSON object");
        cJSON_Delete(root);
        return;
    }

    const cJSON *action_item = cJSON_GetObjectItemCaseSensitive(root, "action");
    if (!cJSON_IsNumber(action_item)) {
        LOG_WARN("MQTT message ignored: missing or invalid 'action'");
        cJSON_Delete(root);
        return;
    }

    int action = action_item->valueint;
    const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(root, "val");
    int val = cJSON_IsNumber(val_item) ? val_item->valueint : 0;
    
    const cJSON *task_id_item = cJSON_GetObjectItemCaseSensitive(root, "task_id");
    const char *task_id = cJSON_IsString(task_id_item) ? task_id_item->valuestring : "";

    LOG_INFOF("Parsed MQTT Command: task_id=%s, action=%d, val=%d", task_id, action, val);

    // 兼容现有的 action_handler 签名，将 task_id 作为备用参数传递下去
    mqtt_message_handle_action(action, task_id);

    cJSON_Delete(root);
}

static void mqtt_message_task_entry(void *arg)
{
    (void)arg;
    mqtt_message_item_t item = {0};

    while (true) {
        if (xQueueReceive(s_mqtt_message_queue, &item, portMAX_DELAY) == pdTRUE) {
            mqtt_message_process_json(item.topic, item.json);
            mqtt_message_free_item(&item);

            // 当前 MQTT 消息解析与分发完成，释放早锁定 (Early Lock) 的锁
            system_wake_lock_release();
        }
    }
}

esp_err_t start_mqtt_message_task(void)
{
    if (s_mqtt_message_queue == NULL) {
        s_mqtt_message_queue = xQueueCreate(MQTT_MESSAGE_QUEUE_LEN, sizeof(mqtt_message_item_t));
        if (s_mqtt_message_queue == NULL) {
            LOG_ERROR("Failed to create MQTT message queue");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_mqtt_message_task != NULL) {
        return ESP_OK;
    }

    if (xTaskCreate(mqtt_message_task_entry,
                    "mqtt_message",
                    MQTT_MESSAGE_TASK_STACK_SIZE,
                    NULL,
                    4,
                    &s_mqtt_message_task) != pdPASS) {
        LOG_ERROR("Failed to create MQTT message task");
        return ESP_ERR_INVALID_STATE;
    }

    LOG_INFO("MQTT message task started");
    return ESP_OK;
}

esp_err_t mqtt_message_task_submit(const char *topic, const uint8_t *data, size_t len)
{
    if (s_mqtt_message_queue == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > MQTT_MESSAGE_MAX_JSON_LEN) {
        LOG_WARNF("MQTT message ignored: JSON too large len=%u", (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    // 早锁定 (Early Lock)：只要收到网络消息就立刻上锁，防止休眠管理器在此刻断网
    system_wake_lock_acquire();

    mqtt_message_item_t item = {0};
    item.json = calloc(1, len + 1);
    if (item.json == NULL) {
        system_wake_lock_release();
        return ESP_ERR_NO_MEM;
    }
    memcpy(item.json, data, len);
    item.len = len;

    if (topic != NULL && topic[0] != '\0') {
        size_t topic_len = strlen(topic);
        item.topic = malloc(topic_len + 1);
        if (item.topic == NULL) {
            mqtt_message_free_item(&item);
            system_wake_lock_release();
            return ESP_ERR_NO_MEM;
        }
        memcpy(item.topic, topic, topic_len + 1);
    }

    if (xQueueSend(s_mqtt_message_queue, &item, 0) != pdTRUE) {
        mqtt_message_free_item(&item);
        system_wake_lock_release();
        LOG_WARN("MQTT message queue full, dropping message");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void mqtt_message_task_register_action_handler(mqtt_action_handler_t handler)
{
    s_action_handler = handler;
}
