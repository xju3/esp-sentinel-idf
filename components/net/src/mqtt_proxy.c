#include "mqtt_proxy.h"
#include "mqtt_client.h"
#include "config_manager.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <errno.h>
#include <string.h>

// 全局 MQTT 客户端句柄
esp_mqtt_client_handle_t g_mqtt_client = NULL;
static mqtt_proxy_event_cb_t s_mqtt_proxy_event_cb = NULL;
static void *s_mqtt_proxy_event_user_ctx = NULL;

// MQTT 事件处理函数
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base, 
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        LOG_INFO("MQTT connected");
        if (s_mqtt_proxy_event_cb)
        {
            s_mqtt_proxy_event_cb(MQTT_PROXY_EVENT_READY, event ? event->msg_id : -1, s_mqtt_proxy_event_user_ctx);
        }
        // 订阅主题（如果需要）
        // esp_mqtt_client_subscribe(g_mqtt_client, "/device/command", 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        LOG_WARN("MQTT disconnected");
        if (s_mqtt_proxy_event_cb)
        {
            s_mqtt_proxy_event_cb(MQTT_PROXY_EVENT_DISCONNECTED, event ? event->msg_id : -1, s_mqtt_proxy_event_user_ctx);
        }
        break;

    case MQTT_EVENT_SUBSCRIBED:
        LOG_INFO("MQTT subscribed");
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        LOG_INFO("MQTT unsubscribed");
        break;

    case MQTT_EVENT_PUBLISHED:
        if (s_mqtt_proxy_event_cb)
        {
            s_mqtt_proxy_event_cb(MQTT_PROXY_EVENT_PUBLISHED, event ? event->msg_id : -1, s_mqtt_proxy_event_user_ctx);
        }
        break;

    case MQTT_EVENT_DATA:
        LOG_INFOF("MQTT data received: topic=%.*s, data=%.*s",
                  event->topic_len, event->topic,
                  event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        LOG_ERROR("MQTT error");
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
        if (s_mqtt_proxy_event_cb)
        {
            s_mqtt_proxy_event_cb(MQTT_PROXY_EVENT_ERROR, event ? event->msg_id : -1, s_mqtt_proxy_event_user_ctx);
        }
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

// 初始化 MQTT 客户端
esp_err_t init_mqtt_client(void)
{
    if (g_mqtt_client != NULL)
    {
        LOG_WARN("MQTT client already initialized");
        return ESP_OK;
    }

    // 检查 MQTT 服务器地址是否配置
    if (strlen(g_user_config.host) == 0)
    {
        LOG_ERROR("MQTT server address not configured");
        return ESP_FAIL;
    }

    LOG_INFOF("Initializing MQTT client to: %s", g_user_config.host);

    // 构建完整的 MQTT URI
    char mqtt_uri[LEN_MAX_HOST + 20]; // 额外空间用于协议和端口
    if (strstr(g_user_config.host, "://") == NULL)
    {
        // 如果没有协议前缀，添加 mqtt://
        snprintf(mqtt_uri, sizeof(mqtt_uri), "mqtt://%s:1883", g_user_config.host);
    }
    else
    {
        // 如果已有协议前缀，直接使用
        strncpy(mqtt_uri, g_user_config.host, sizeof(mqtt_uri) - 1);
        mqtt_uri[sizeof(mqtt_uri) - 1] = '\0';
    }

    LOG_INFOF("MQTT URI: %s", mqtt_uri);

    // 配置 MQTT 客户端
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_uri,
        .credentials.client_id = g_user_config.device_id,
        .session.keepalive = 60,                 // 60秒心跳
        .network.disable_auto_reconnect = false, // 启用自动重连
        .network.reconnect_timeout_ms = 5000,    // 5秒重连间隔
    };

    // 创建 MQTT 客户端
    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (g_mqtt_client == NULL)
    {
        LOG_ERROR("Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // 注册事件处理器
    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // 启动 MQTT 客户端
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

// 停止 MQTT 客户端
esp_err_t mqtt_client_stop(void)
{
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
    LOG_INFO("MQTT client stopped");
    return ESP_OK;
}
