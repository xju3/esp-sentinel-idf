#include "config_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "fs_utils.h"

#define CONFIG_LOG_CHUNK 512

static const char *TAG = "config_manager";

static void safe_copy(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t len = strnlen(src, dst_size - 1);
    memcpy(dst, src, len);
    dst[len] = '\0';
}

esp_err_t config_manager_init(void)
{
    esp_err_t err = ESP_OK;

    if (!fsu_is_storage_mounted()) {
        err = fsu_mount_storage(false);
    }
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    if (!fsu_is_user_mounted()) {
        (void)fsu_mount_user(true);
    }

    return ESP_OK;
}

static void apply_json_to_config(user_config_t *cfg, const cJSON *root)
{
    if (!cfg || !root) {
        return;
    }

    const cJSON *item = NULL;

    item = cJSON_GetObjectItemCaseSensitive(root, "deviceId");
    if (cJSON_IsString(item)) {
        safe_copy(cfg->device_id, sizeof(cfg->device_id), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "deviceName");
    if (cJSON_IsString(item)) {
        safe_copy(cfg->device_name, sizeof(cfg->device_name), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "modelType");
    if (cJSON_IsString(item)) {
        safe_copy(cfg->device_type, sizeof(cfg->device_type), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "years");
    if (cJSON_IsNumber(item)) {
        cfg->years = (int16_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "comm_type");
    if (cJSON_IsNumber(item)) {
        cfg->comm_type = (int8_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "configured");
    if (cJSON_IsBool(item)) {
        cfg->is_configured = cJSON_IsTrue(item);
    }

    const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
    if (cJSON_IsObject(wifi)) {
        const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(wifi, "ssid");
        if (cJSON_IsString(ssid)) {
            safe_copy(cfg->wifi.ssid, sizeof(cfg->wifi.ssid), ssid->valuestring);
        }
        const cJSON *pass = cJSON_GetObjectItemCaseSensitive(wifi, "pass");
        if (cJSON_IsString(pass)) {
            safe_copy(cfg->wifi.pass, sizeof(cfg->wifi.pass), pass->valuestring);
        }
    }

    const cJSON *host = cJSON_GetObjectItemCaseSensitive(root, "host");
    if (cJSON_IsObject(host)) {
        const cJSON *http = cJSON_GetObjectItemCaseSensitive(host, "http");
        if (cJSON_IsString(http)) {
            safe_copy(cfg->host.http, sizeof(cfg->host.http), http->valuestring);
        }
        const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(host, "mqtt");
        if (cJSON_IsString(mqtt)) {
            safe_copy(cfg->host.mqtt, sizeof(cfg->host.mqtt), mqtt->valuestring);
        }
    }

    const cJSON *detect = cJSON_GetObjectItemCaseSensitive(root, "detect");
    if (cJSON_IsObject(detect)) {
        const cJSON *type = cJSON_GetObjectItemCaseSensitive(detect, "type");
        if (cJSON_IsNumber(type)) {
            cfg->detect.type = (int16_t)type->valueint;
        }
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(detect, "value");
        if (cJSON_IsNumber(value)) {
            cfg->detect.value = (int32_t)value->valueint;
        }
    }

    const cJSON *report = cJSON_GetObjectItemCaseSensitive(root, "report");
    if (cJSON_IsObject(report)) {
        const cJSON *type = cJSON_GetObjectItemCaseSensitive(report, "type");
        if (cJSON_IsNumber(type)) {
            cfg->report.type = (int16_t)type->valueint;
        }
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(report, "value");
        if (cJSON_IsNumber(value)) {
            cfg->report.value = (int32_t)value->valueint;
        }
    }
}

static void parser_apply_wrapper(cJSON *root, void *ctx)
{
    apply_json_to_config((user_config_t *)ctx, root);
}

static esp_err_t load_and_apply(const char *path, user_config_t *cfg)
{
    return fsu_parse_json(path, parser_apply_wrapper, cfg);
}

esp_err_t config_manager_log_default_json(void)
{
    if (config_manager_init() != ESP_OK) {
        return ESP_FAIL;
    }

    size_t len = 0;
    char *json = fsu_read_file_alloc(CONFIG_DEFAULT_PATH, &len);
    if (!json) {
        ESP_LOGE(TAG, "Default config not found: %s", CONFIG_DEFAULT_PATH);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Default config (len=%u):", (unsigned)len);
    for (size_t i = 0; i < len; i += CONFIG_LOG_CHUNK) {
        size_t chunk = (len - i > CONFIG_LOG_CHUNK) ? CONFIG_LOG_CHUNK : (len - i);
        ESP_LOGI(TAG, "%.*s", (int)chunk, json + i);
    }

    free(json);
    return ESP_OK;
}

// load system configuration if user does not configured.
esp_err_t config_manager_load(user_config_t *out_cfg)
{
    if (!out_cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config_manager_init() != ESP_OK) {
        return ESP_FAIL;
    }

    memset(out_cfg, 0, sizeof(*out_cfg));

    esp_err_t err = load_and_apply(CONFIG_DEFAULT_PATH, out_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load default config: %s", CONFIG_DEFAULT_PATH);
        return err;
    }

    if (fsu_is_user_mounted() && fsu_file_exists(CONFIG_USER_PATH)) {
        err = load_and_apply(CONFIG_USER_PATH, out_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "User config parse failed; using defaults");
            (void)config_manager_log_default_json();
            out_cfg->is_configured = false;
            return ESP_OK;
        }
        return ESP_OK;
    }

    ESP_LOGI(TAG, "User config not found; using defaults");
    (void)config_manager_log_default_json();
    out_cfg->is_configured = false;
    return ESP_OK;
}

esp_err_t config_manager_save_user_json(const char *json)
{
    if (!json) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config_manager_init() != ESP_OK) {
        return ESP_FAIL;
    }

    if (!fsu_is_user_mounted()) {
        ESP_LOGE(TAG, "User SPIFFS not mounted");
        return ESP_FAIL;
    }

    return fsu_write_file(CONFIG_USER_PATH, json, strlen(json));
}

esp_err_t config_manager_save_user(const user_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "deviceId", cfg->device_id);
    cJSON_AddStringToObject(root, "deviceName", cfg->device_name);
    cJSON_AddStringToObject(root, "modelType", cfg->device_type);
    cJSON_AddNumberToObject(root, "years", cfg->years);
    cJSON_AddNumberToObject(root, "comm_type", cfg->comm_type);
    cJSON_AddBoolToObject(root, "configured", cfg->is_configured);

    cJSON *wifi = cJSON_CreateObject();
    cJSON_AddStringToObject(wifi, "ssid", cfg->wifi.ssid);
    cJSON_AddStringToObject(wifi, "pass", cfg->wifi.pass);
    cJSON_AddItemToObject(root, "wifi", wifi);

    cJSON *host = cJSON_CreateObject();
    cJSON_AddStringToObject(host, "http", cfg->host.http);
    cJSON_AddStringToObject(host, "mqtt", cfg->host.mqtt);
    cJSON_AddItemToObject(root, "host", host);

    cJSON *detect = cJSON_CreateObject();
    cJSON_AddNumberToObject(detect, "type", cfg->detect.type);
    cJSON_AddNumberToObject(detect, "value", cfg->detect.value);
    cJSON_AddItemToObject(root, "detect", detect);

    cJSON *report = cJSON_CreateObject();
    cJSON_AddNumberToObject(report, "type", cfg->report.type);
    cJSON_AddNumberToObject(report, "value", cfg->report.value);
    cJSON_AddItemToObject(root, "report", report);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = config_manager_save_user_json(json);
    free(json);
    return err;
}
