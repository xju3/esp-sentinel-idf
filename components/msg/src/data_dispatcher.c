#include "data_dispatcher.h"
#include "config_manager.h"
#include "logger.h"
#include "mqtt_client.h"
#include "msg_sentinel.pb-c.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

QueueHandle_t g_msg_dispatcher_queue = NULL;

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
    MsgPayload msg;

    LOG_INFO("Data dispatcher started. Ready to forward MsgPayload to MQTT");
    while (1)
    {
        if (g_msg_dispatcher_queue == NULL)
        {
            return;
        }

        // 阻塞等待队列消息
        if (xQueueReceive(g_msg_dispatcher_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            // 在 dispatcher 统一补填 sn 和 ts，减少各处重复赋值
            if (msg.sn == 0)
            {
                msg.sn = SN;
            }
            if (msg.ts == 0)
            {
                msg.ts = esp_timer_get_time() / 1000; // us -> ms
            }

            // 序列化 MsgPayload
            size_t packed_size = msg_payload__get_packed_size(&msg);
            uint8_t *buffer = malloc(packed_size);
            if (buffer == NULL)
            {
                LOG_ERROR("Failed to allocate buffer for MsgPayload serialization");
                continue;
            }

            msg_payload__pack(&msg, buffer);

            // 发送到 MQTT，topic 固定为 "sentinel/machine_status"
            bool success = send_binary_to_mqtt("sentinel", buffer, packed_size);

            free(buffer);
            free(msg.data.data);

            if (!success)
            {
                LOG_WARN("Failed to send MsgPayload to MQTT, dropping message");
            }
        }
    }
}

esp_err_t send_protobuf_message(uint32_t event_type, const ProtobufCMessage *message)
{
    if (g_msg_dispatcher_queue == NULL || message == NULL)
    {
        LOG_ERROR("Message dispatcher queue not initialized or invalid message");
        return ESP_ERR_INVALID_STATE;
    }

    // Pack the protobuf message
    size_t packed_size = protobuf_c_message_get_packed_size(message);
    uint8_t *buffer = malloc(packed_size);
    if (buffer == NULL)
    {
        LOG_ERROR("Failed to allocate buffer for message packing");
        return ESP_ERR_NO_MEM;
    }
    protobuf_c_message_pack(message, buffer);

    // Create MsgPayload
    MsgPayload payload = MSG_PAYLOAD__INIT;
    payload.et = event_type;
    payload.data.len = packed_size;
    payload.data.data = buffer;

    // Send to queue
    BaseType_t result = xQueueSend(g_msg_dispatcher_queue, &payload, 0);

    if (result != pdTRUE)
    {
        free(buffer);
        LOG_WARN("Failed to send message to dispatcher queue");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t data_dispatcher_start(void)
{
    if (!g_msg_dispatcher_queue)
    {
        // 如果队列未初始化，则创建一个
        g_msg_dispatcher_queue = xQueueCreate(20, sizeof(MsgPayload));
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
