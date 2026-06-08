#include "data_dispatcher.h"

#include "logger.h"

QueueHandle_t g_msg_dispatcher_queue = NULL;

#define DISPATCHER_INIT_QUEUE_LEN 1

esp_err_t send_protobuf_message(uint32_t event_type, const ProtobufCMessage *message)
{
    (void)event_type;
    (void)message;

    LOG_WARN("Legacy protobuf dispatcher is disabled; report JSON is uploaded via HTTP.");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t data_dispatcher_start(void)
{
    if (!g_msg_dispatcher_queue)
    {
        uint8_t init_marker = 0;
        g_msg_dispatcher_queue = xQueueCreate(DISPATCHER_INIT_QUEUE_LEN, sizeof(init_marker));
        if (!g_msg_dispatcher_queue)
        {
            LOG_ERROR("Failed to create message dispatcher init marker!");
            return ESP_FAIL;
        }
    }

    LOG_INFO("Data dispatcher legacy marker ready; protobuf transport is disabled.");
    return ESP_OK;
}
