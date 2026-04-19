#ifndef MQTT_PROXY_H_
#define MQTT_PROXY_H_

#include "esp_err.h"
#include "esp_event.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for MQTT_PROXY client handle
typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

typedef enum
{
    MQTT_PROXY_EVENT_READY = 0,
    MQTT_PROXY_EVENT_PUBLISHED = 1,
    MQTT_PROXY_EVENT_DISCONNECTED = 2,
    MQTT_PROXY_EVENT_ERROR = 3,
} mqtt_proxy_event_t;

typedef void (*mqtt_proxy_event_cb_t)(mqtt_proxy_event_t event, int32_t msg_id, void *user_ctx);

/**
 * @brief 全局 MQTT_PROXY 客户端句柄
 * * 用于其他模块发送 MQTT_PROXY 消息
 */
extern esp_mqtt_client_handle_t g_mqtt_client;

/**
 * @brief 初始化 MQTT_PROXY 客户端
 * * 在网络连接成功后调用，启动 MQTT_PROXY 客户端
 * * @return esp_err_t 返回 ESP_OK 表示初始化成功
 */
esp_err_t init_mqtt_client(void);
esp_err_t mqtt_client_stop(void);
void mqtt_proxy_set_event_callback(mqtt_proxy_event_cb_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_PROXY_H_ */
