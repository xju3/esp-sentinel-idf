#include "data_dispatcher.h"
#include "config_manager.h"
#include "logger.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

QueueHandle_t g_msg_dispatcher_queue = NULL;
binary_msg_t g_binary_msg = {0};

// 全局 MQTT 句柄 (在 network_manager 中定义和初始化)
extern esp_mqtt_client_handle_t g_mqtt_client;

// 发送重试配置
#define MAX_SEND_RETRIES 3
#define RETRY_DELAY_MS 1000

// 检查 MQTT 连接状态
static bool is_mqtt_connected(void)
{
    return g_mqtt_client != NULL;
}

// 发送二进制消息到 MQTT (带重试机制)
static bool send_binary_to_mqtt(const char *topic, const void *payload, size_t len)
{
    if (!is_mqtt_connected() || payload == NULL || len == 0)
    {
        LOG_WARN("MQTT not connected or invalid payload");
        return false;
    }

    // 注意：QoS 设置为 1，确保 Broker 至少收到一次
    // esp_mqtt_client_publish 的参数：client, topic, data, len, qos, retain
    for (int retry = 0; retry < MAX_SEND_RETRIES; retry++)
    {
        int msg_id = esp_mqtt_client_publish(g_mqtt_client, topic, (const char *)payload, len, 1, 0);

        if (msg_id != -1)
        {
            // 对于二进制数据，记录长度比记录内容更有意义
            // LOG_INFOF("Binary data sent: %d bytes (msg_id: %d)", len, msg_id);
            return true;
        }

        if (retry < MAX_SEND_RETRIES - 1)
        {
            LOG_WARNF("MQTT binary publish failed, retry %d/%d", retry + 1, MAX_SEND_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }

    LOG_ERROR("MQTT binary publish failed after retries");
    return false;
}

static void dispatcher_task(void *arg)
{
    binary_msg_t msg;

    LOG_INFO("Data dispatcher started. Ready to forward binary data to MQTT");
    while (1)
    {
        if (g_msg_dispatcher_queue == NULL)
        {
            return;
        }

        // 阻塞等待队列消息 (真正的消费者)
        if (xQueueReceive(g_msg_dispatcher_queue, &msg, portMAX_DELAY) == pdTRUE)
        {

            if (!msg.data || msg.len == 0)
            {
                LOG_WARN("Invalid message: data is NULL or length is 0");
                continue;
            }

            // 直接发送二进制数据到 MQTT
            bool success = send_binary_to_mqtt(msg.topic, msg.data, msg.len);

            if (!success)
            {
                // 注意：这里选择丢弃消息而不是重新入队，因为：
                // 1. 保持实时性：旧数据可能已过时
                // 2. 避免队列积压：网络恢复后会有新数据
                // 3. 简化设计：dispatcher 无需关心数据内容
                LOG_WARNF("Failed to send binary data to %s, dropping message", msg.topic);
            }
        }
    }
}

esp_err_t data_dispatcher_start(void)
{
    if (!g_msg_dispatcher_queue)
    {
        // 如果队列未初始化，则创建一个
        g_msg_dispatcher_queue = xQueueCreate(20, sizeof(binary_msg_t));
        if (!g_msg_dispatcher_queue)
        {
            LOG_ERROR("Failed to create message dispatcher queue!");
            return ESP_FAIL;
        }
        LOG_INFO("Created message dispatcher queue");
    }

    xTaskCreate(dispatcher_task, "data_dispatcher", 4096, NULL, 5, NULL);
    LOG_INFO("Data dispatcher task created");
    return ESP_OK;
}
