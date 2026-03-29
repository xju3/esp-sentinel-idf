#ifndef MQTT_PROXY_H_
#define MQTT_PROXY_H_

#include "esp_err.h"
#include "esp_event.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for MQTT_PROXY client handle
typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

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

#ifdef __cplusplus
}
#endif

#endif /* MQTT_PROXY_H_ */
