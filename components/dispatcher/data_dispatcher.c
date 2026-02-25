#include "data_dispatcher.h"
#include "task_monitor.h"
#include "config_manager.h"
#include "logger.h"
#include "cJSON.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// 全局 MQTT 句柄 (在 network_manager 中定义和初始化)
extern esp_mqtt_client_handle_t g_mqtt_client;

// 发送重试配置
#define MAX_SEND_RETRIES 3
#define RETRY_DELAY_MS 1000

// 检查 MQTT 连接状态
static bool is_mqtt_connected(void) {
    return g_mqtt_client != NULL;
}

// 构建 JSON 消息
static char* build_json_message(const monitor_rms_msg_t *msg) {
    if (!msg) return NULL;
    
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;
    
    cJSON_AddStringToObject(root, "deviceId", g_user_config.device_id);
    cJSON_AddNumberToObject(root, "ts", (double)msg->payload.rms.timestamp);
    
    if (msg->type == MONITOR_MSG_TYPE_RMS) {
        cJSON_AddStringToObject(root, "type", "rms");
        cJSON *data = cJSON_CreateObject();
        cJSON_AddNumberToObject(data, "x", msg->payload.rms.rms_x);
        cJSON_AddNumberToObject(data, "y", msg->payload.rms.rms_y);
        cJSON_AddNumberToObject(data, "z", msg->payload.rms.rms_z);
        cJSON_AddItemToObject(root, "data", data);
    }
    // 未来可以添加其他消息类型处理，如 FFT
    // else if (msg->type == MONITOR_MSG_TYPE_FFT) { ... }
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    return json_str;
}

// 发送消息到 MQTT (带重试机制)
static bool send_to_mqtt(const char *topic, const char *json_str) {
    if (!is_mqtt_connected()) {
        LOG_WARN("MQTT client not connected");
        return false;
    }
    
    for (int retry = 0; retry < MAX_SEND_RETRIES; retry++) {
        int msg_id = esp_mqtt_client_publish(g_mqtt_client, topic, json_str, 0, 1, 0);
        
        if (msg_id != -1) {
            // LOG_INFOF("Data sent successfully (msg_id: %d)", msg_id);
            return true;
        }
        
        if (retry < MAX_SEND_RETRIES - 1) {
            LOG_WARNF("MQTT publish failed, retry %d/%d in %dms", 
                     retry + 1, MAX_SEND_RETRIES, RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }
    
    LOG_ERROR("MQTT publish failed after retries");
    return false;
}

static void dispatcher_task(void *arg) {
    monitor_rms_msg_t msg;
    char topic[128];
    
    // 构造 MQTT Topic: /device/{id}/data
    snprintf(topic, sizeof(topic), "/device/%s/data", g_user_config.device_id);
    LOG_INFOF("Data dispatcher started. Topic: %s", topic);

    while (1) {
        // 阻塞等待队列消息 (真正的消费者)
        if (xQueueReceive(g_monitor_rms_queue, &msg, portMAX_DELAY) == pdTRUE) {
            
            // 构建 JSON 消息
            char *json_str = build_json_message(&msg);
            if (!json_str) {
                LOG_ERROR("Failed to build JSON message");
                continue;
            }
            
            // 发送到 MQTT (网络透明性：不关心底层是 4G 还是 WiFi)
            bool success = send_to_mqtt(topic, json_str);
            
            if (!success) {
                LOG_WARN("Failed to send data, dropping message");
                // 注意：这里选择丢弃消息而不是重新入队，因为：
                // 1. 保持实时性：旧数据可能已过时
                // 2. 避免队列积压：网络恢复后会有新数据
                // 3. 简化设计：符合黑盒模式原则
            }
            
            free(json_str);
        }
    }
}

esp_err_t data_dispatcher_start(void) {
    if (!g_monitor_rms_queue) {
        LOG_ERROR("Monitor queue not initialized!");
        return ESP_FAIL;
    }
    
    xTaskCreate(dispatcher_task, "data_dispatcher", 4096, NULL, 5, NULL);
    LOG_INFO("Data dispatcher task created");
    return ESP_OK;
}
