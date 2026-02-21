#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_MAX_DEVICE_ID_LEN   64
#define CONFIG_MAX_DEVICE_NAME_LEN 64
#define CONFIG_MAX_DEVICE_TYPE_LEN 16
#define CONFIG_MAX_WIFI_SSID_LEN   64
#define CONFIG_MAX_WIFI_PASS_LEN   64
#define CONFIG_MAX_HOST_HTTP_LEN   128
#define CONFIG_MAX_HOST_MQTT_LEN   128

#define CONFIG_DEFAULT_PATH "/system/config/default_config.json"
#define CONFIG_USER_PATH    "/user/user_config.json"

typedef struct {
    int16_t type;
    int32_t value;
} freq_config_t;

typedef struct {
    char ssid[CONFIG_MAX_WIFI_SSID_LEN];
    char pass[CONFIG_MAX_WIFI_PASS_LEN];
} user_wifi_config_t;

typedef struct {
    char http[CONFIG_MAX_HOST_HTTP_LEN];
    char mqtt[CONFIG_MAX_HOST_MQTT_LEN];
} host_config_t;

typedef struct {
    char device_id[CONFIG_MAX_DEVICE_ID_LEN];
    char device_name[CONFIG_MAX_DEVICE_NAME_LEN];
    char device_type[CONFIG_MAX_DEVICE_TYPE_LEN];
    int8_t comm_type;
    int16_t years;
    freq_config_t detect;
    freq_config_t report;
    user_wifi_config_t wifi;
    host_config_t host;
    bool is_configured;
} user_config_t;

// 全局配置实例（加载后可被各模块直接使用）
extern user_config_t g_user_config;

esp_err_t config_manager_init(void);

// Load default config, then override with user config if present.
esp_err_t config_manager_load(user_config_t *out_cfg);

// Save user config as JSON string to user partition.
esp_err_t config_manager_save_user_json(const char *json);

// Save user config struct as JSON to user partition.
esp_err_t config_manager_save_user(const user_config_t *cfg);

// Log the default JSON config (used when user config is missing or invalid).
esp_err_t config_manager_log_default_json(void);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
