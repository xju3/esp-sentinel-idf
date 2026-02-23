#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEN_MAX_DEVICE_ID   64
#define LEN_MAX_DEVICE_NAME 64
#define LEN_MAX_DEVICE_TYPE 32
#define LEN_MAX_HOST 32
#define LEN_MAX_WIFI_SSID   64
#define LEN_MAX_WIFI_PASS   64
#define LEN_MAX_HOST_HTTP   128
#define LEN_MAX_ISO_STD     16
#define LEN_MAX_ISO_CAT     32
#define LEN_MAX_ISO_FOUND   16

#define FILE_PATH_CONFIG_DEFAULT "/system/c/default_config.json"
#define FILE_PATH_HW_CONSUMPTION "/system/c/consumption.json"
#define FILE_PATH_CONFIG_USER    "/user/user_config.json"
#define FILE_PATH_DEVICE_PROFILE     "/user/device_profile.json"


typedef struct {
    int16_t type;
    int32_t value;
    int32_t cycle;   // 上报周期倍数（仅 report 使用；detect 置 1）
} freq_config_t;

typedef struct {
    int8_t standard;    // 1: ISO10816, 2: ISO20816
    int8_t category;    // 具体类别（class2, pump, vertical...）
    int8_t foundation;  // 1: rigid, 2: flexible
} iso_config_t;

typedef struct {
    char ssid[LEN_MAX_WIFI_SSID];
    char pass[LEN_MAX_WIFI_PASS];
} user_wifi_config_t;

typedef struct {
    char device_id[LEN_MAX_DEVICE_ID];
    char device_name[LEN_MAX_DEVICE_NAME];
    char device_type[LEN_MAX_DEVICE_TYPE];
    char host[LEN_MAX_HOST];
    int32_t rpm;
    int8_t network;
    int16_t months;
    int16_t battery;
    bool ble;
    iso_config_t iso;
    int16_t detect;
    int16_t report;
    user_wifi_config_t wifi;
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
