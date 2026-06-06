#include "data_dispatcher.h"
#include "bsp_wifi.h"
#include "config_manager.h"
#include "logger.h"
#include "mqtt_client.h"
#include "mqtt_proxy.h"
#include "msg_sentinel.pb-c.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stdlib.h>

QueueHandle_t g_msg_dispatcher_queue = NULL;

extern esp_mqtt_client_handle_t g_mqtt_client;

#define DISPATCHER_INIT_QUEUE_LEN 1
#define DISPATCHER_WIFI_PUBLISH_ACK_TIMEOUT_MS 5000

static volatile bool s_transport_ready = false;
static volatile bool s_transport_failed = false;
static volatile bool s_publish_ack = false;

static bool is_mqtt_connected(void)
{
    if (g_user_config.network == 1)
    {
        return s_transport_ready;
    }
    return g_mqtt_client != NULL && s_transport_ready;
}

static esp_err_t dispatcher_pack_payload(const MsgPayload *msg, uint8_t **out_data, size_t *out_len)
{
    if (!msg || !out_data || !out_len)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out_data = NULL;
    *out_len = 0;

    MsgPayload normalized = *msg;
    if (normalized.sn == 0)
    {
        normalized.sn = SN;
    }

    size_t packed_size = msg_payload__get_packed_size(&normalized);
    uint8_t *buffer = malloc(packed_size);
    if (buffer == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    msg_payload__pack(&normalized, buffer);
    *out_data = buffer;
    *out_len = packed_size;
    return ESP_OK;
}

static void dispatcher_network_channel_established(void)
{
    if (g_user_config.network == 1)
    {
        s_transport_ready = true;
        return;
    }

    esp_err_t err = init_mqtt_client();
    if (err != ESP_OK)
    {
        LOG_ERRORF("MQTT initialization failed after network bring-up: %s", esp_err_to_name(err));
        s_transport_failed = true;
    }
}

static void dispatcher_mqtt_event_handler(mqtt_proxy_event_t event, int32_t msg_id, void *user_ctx)
{
    (void)msg_id;
    (void)user_ctx;

    switch (event)
    {
        case MQTT_PROXY_EVENT_READY:
            s_transport_ready = true;
            s_transport_failed = false;
            break;
        case MQTT_PROXY_EVENT_PUBLISHED:
            s_publish_ack = true;
            break;
        case MQTT_PROXY_EVENT_DISCONNECTED:
        case MQTT_PROXY_EVENT_ERROR:
            s_transport_failed = true;
            s_transport_ready = false;
            break;
        default:
            break;
    }
}

static esp_err_t dispatcher_request_transport(void)
{
    s_transport_ready = false;
    s_transport_failed = false;
    s_publish_ack = false;

    if (g_user_config.network == 1)
    {
        LOG_INFO("Dispatcher bringing up 4G transport via proxy");
        esp_err_t err = init_mqtt_client();
        if (err == ESP_OK)
        {
            s_transport_ready = true;
        }
        return err;
    }

    LOG_INFO("Dispatcher bringing up WiFi STA transport before send");
    return wifi_init_sta(g_user_config.wifi.ssid,
                         g_user_config.wifi.pass,
                         dispatcher_network_channel_established);
}

static void dispatcher_shutdown_transport(void)
{
    (void)mqtt_client_disconnect();
    s_transport_ready = false;
    s_transport_failed = false;
    s_publish_ack = false;
}

static esp_err_t dispatcher_wait_for_transport_ready(void)
{
    esp_err_t err = dispatcher_request_transport();
    if (err != ESP_OK)
    {
        LOG_ERRORF("Transport bring-up start failed: %s", esp_err_to_name(err));
        return err;
    }

    int64_t deadline_us = esp_timer_get_time() +
                          ((int64_t)CONFIG_SENTINEL_NETWORK_BRINGUP_TIMEOUT_SEC * 1000000LL);
    while (esp_timer_get_time() < deadline_us)
    {
        if (is_mqtt_connected())
        {
            return ESP_OK;
        }
        if (s_transport_failed)
        {
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    LOG_WARN("Timed out waiting for network/MQTT bring-up");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t dispatcher_wait_for_wifi_publish_ack(void)
{
    if (g_user_config.network == 1)
    {
        return ESP_OK;
    }

    int64_t deadline_us = esp_timer_get_time() +
                          ((int64_t)DISPATCHER_WIFI_PUBLISH_ACK_TIMEOUT_MS * 1000LL);
    while (esp_timer_get_time() < deadline_us)
    {
        if (s_publish_ack)
        {
            return ESP_OK;
        }
        if (s_transport_failed)
        {
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    LOG_WARN("Timed out waiting for MQTT publish acknowledgement");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t dispatcher_send_payload_now(const MsgPayload *msg)
{
    uint8_t *data = NULL;
    size_t len = 0;
    esp_err_t err = dispatcher_pack_payload(msg, &data, &len);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to pack dispatcher payload: %s", esp_err_to_name(err));
        return err;
    }

    err = dispatcher_wait_for_transport_ready();
    if (err != ESP_OK)
    {
        free(data);
        dispatcher_shutdown_transport();
        return err;
    }

    s_publish_ack = false;
    err = mqtt_proxy_publish("sentinel", data, len, 1, 0);
    if (err == ESP_OK)
    {
        err = dispatcher_wait_for_wifi_publish_ack();
    }

    free(data);
    dispatcher_shutdown_transport();

    if (err != ESP_OK)
    {
        LOG_WARNF("Dispatcher publish failed: %s", esp_err_to_name(err));
        return err;
    }

    LOG_INFO("Dispatcher publish completed");
    return ESP_OK;
}

esp_err_t send_protobuf_message(uint32_t event_type, const ProtobufCMessage *message)
{
    if (g_msg_dispatcher_queue == NULL || message == NULL)
    {
        LOG_ERROR("Message dispatcher queue not initialized or invalid message");
        return ESP_ERR_INVALID_STATE;
    }

    size_t packed_size = protobuf_c_message_get_packed_size(message);
    uint8_t *buffer = malloc(packed_size);
    if (buffer == NULL)
    {
        LOG_ERROR("Failed to allocate buffer for message packing");
        return ESP_ERR_NO_MEM;
    }
    protobuf_c_message_pack(message, buffer);

    MsgPayload payload = MSG_PAYLOAD__INIT;
    payload.et = event_type;
    payload.data.len = packed_size;
    payload.data.data = buffer;

    esp_err_t err = dispatcher_send_payload_now(&payload);
    free(buffer);
    return err;
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
        LOG_INFO("Created message dispatcher init marker");
    }

    mqtt_proxy_set_event_callback(dispatcher_mqtt_event_handler, NULL);
    LOG_INFO("Data dispatcher ready. Payloads will be sent synchronously");
    return ESP_OK;
}
