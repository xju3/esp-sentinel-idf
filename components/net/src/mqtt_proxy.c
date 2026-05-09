#include "mqtt_proxy.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "bsp_4g.h"
#include "config_manager.h"
#include "logger.h"
#include "mqtt_client.h"

esp_mqtt_client_handle_t g_mqtt_client = NULL;
static mqtt_proxy_event_cb_t s_mqtt_proxy_event_cb = NULL;
static void *s_mqtt_proxy_event_user_ctx = NULL;
static bool s_mqtt_ready = false;

static void mqtt_proxy_emit_event(mqtt_proxy_event_t event, int32_t msg_id)
{
    if (s_mqtt_proxy_event_cb)
    {
        s_mqtt_proxy_event_cb(event, msg_id, s_mqtt_proxy_event_user_ctx);
    }
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        LOG_INFO("MQTT connected");
        s_mqtt_ready = true;
        mqtt_proxy_emit_event(MQTT_PROXY_EVENT_READY, event ? event->msg_id : -1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        LOG_WARN("MQTT disconnected");
        s_mqtt_ready = false;
        mqtt_proxy_emit_event(MQTT_PROXY_EVENT_DISCONNECTED, event ? event->msg_id : -1);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        LOG_INFO("MQTT subscribed");
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        LOG_INFO("MQTT unsubscribed");
        break;

    case MQTT_EVENT_PUBLISHED:
        mqtt_proxy_emit_event(MQTT_PROXY_EVENT_PUBLISHED, event ? event->msg_id : -1);
        break;

    case MQTT_EVENT_DATA:
        LOG_INFOF("MQTT data received: topic=%.*s, data=%.*s",
                  event->topic_len, event->topic,
                  event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        LOG_ERROR("MQTT error");
        s_mqtt_ready = false;
        if (event && event->error_handle)
        {
            esp_mqtt_error_codes_t *err = event->error_handle;
            LOG_ERRORF("MQTT error detail: type=%d, tls_esp_err=0x%x, tls_stack_err=%d, cert_flags=0x%x, sock_errno=%d (%s), connect_rc=%d",
                       err->error_type,
                       (unsigned int)err->esp_tls_last_esp_err,
                       err->esp_tls_stack_err,
                       (unsigned int)err->esp_tls_cert_verify_flags,
                       err->esp_transport_sock_errno,
                       (err->esp_transport_sock_errno != 0) ? strerror(err->esp_transport_sock_errno) : "n/a",
                       err->connect_return_code);
        }
        mqtt_proxy_emit_event(MQTT_PROXY_EVENT_ERROR, event ? event->msg_id : -1);
        break;

    default:
        break;
    }
}

void mqtt_proxy_set_event_callback(mqtt_proxy_event_cb_t cb, void *user_ctx)
{
    s_mqtt_proxy_event_cb = cb;
    s_mqtt_proxy_event_user_ctx = user_ctx;
}

bool mqtt_proxy_is_ready(void)
{
    return s_mqtt_ready;
}

void mqtt_proxy_notify_ready(void)
{
    s_mqtt_ready = true;
    mqtt_proxy_emit_event(MQTT_PROXY_EVENT_READY, -1);
}

void mqtt_proxy_notify_published(int32_t msg_id)
{
    mqtt_proxy_emit_event(MQTT_PROXY_EVENT_PUBLISHED, msg_id);
}

void mqtt_proxy_notify_disconnected(void)
{
    s_mqtt_ready = false;
    mqtt_proxy_emit_event(MQTT_PROXY_EVENT_DISCONNECTED, -1);
}

void mqtt_proxy_notify_error(void)
{
    s_mqtt_ready = false;
    mqtt_proxy_emit_event(MQTT_PROXY_EVENT_ERROR, -1);
}

int32_t mqtt_proxy_publish(const char *topic, const void *data, size_t len, int qos, int retain)
{
    if (topic == NULL || data == NULL || len == 0 || !mqtt_proxy_is_ready())
    {
        return -1;
    }

    if (g_user_config.network == 1)
    {
        return bsp_4g_mqtt_publish(topic, data, len, qos, retain != 0);
    }

    if (g_mqtt_client == NULL)
    {
        return -1;
    }

    return esp_mqtt_client_publish(g_mqtt_client, topic, (const char *)data, len, qos, retain);
}

esp_err_t init_mqtt_client(void)
{
    if (g_user_config.network == 1)
    {
        return mqtt_proxy_is_ready() ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    if (g_mqtt_client != NULL)
    {
        LOG_WARN("MQTT client already initialized");
        return ESP_OK;
    }

    if (strlen(g_user_config.host) == 0)
    {
        LOG_ERROR("MQTT server address not configured");
        return ESP_FAIL;
    }

    LOG_INFOF("Initializing MQTT client to: %s", g_user_config.host);

    char mqtt_uri[LEN_MAX_HOST + 20];
    if (strstr(g_user_config.host, "://") == NULL)
    {
        snprintf(mqtt_uri, sizeof(mqtt_uri), "mqtt://%s:1883", g_user_config.host);
    }
    else
    {
        strncpy(mqtt_uri, g_user_config.host, sizeof(mqtt_uri) - 1);
        mqtt_uri[sizeof(mqtt_uri) - 1] = '\0';
    }

    LOG_INFOF("MQTT URI: %s", mqtt_uri);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_uri,
        .credentials.client_id = g_user_config.device_id,
        .session.keepalive = 60,
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
    };

    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (g_mqtt_client == NULL)
    {
        LOG_ERROR("Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    esp_err_t err = esp_mqtt_client_start(g_mqtt_client);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to start MQTT client: %d", err);
        g_mqtt_client = NULL;
        return err;
    }

    LOG_INFO("MQTT client started successfully");
    return ESP_OK;
}

esp_err_t mqtt_client_stop(void)
{
    if (g_user_config.network == 1)
    {
        s_mqtt_ready = false;
        return ESP_OK;
    }

    if (g_mqtt_client == NULL)
    {
        return ESP_OK;
    }

    LOG_INFO("Stopping MQTT client");
    esp_err_t err = esp_mqtt_client_stop(g_mqtt_client);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to stop MQTT client: %d", err);
        return err;
    }

    err = esp_mqtt_client_destroy(g_mqtt_client);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to destroy MQTT client: %d", err);
        return err;
    }

    g_mqtt_client = NULL;
    s_mqtt_ready = false;
    LOG_INFO("MQTT client stopped");
    return ESP_OK;
}
