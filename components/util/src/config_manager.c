#include "config_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "fs_utils.h"
#include "logger.h"

#define CONFIG_LOG_CHUNK 512

user_config_t g_user_config;

// 安全拷贝字符串，保证结尾有 '\0' 且不溢出。
static void safe_copy(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0)
    {
        return;
    }
    if (!src)
    {
        dst[0] = '\0';
        return;
    }
    size_t len = strnlen(src, dst_size - 1);
    memcpy(dst, src, len);
    dst[len] = '\0';
}

// 挂载 system/user 分区，为后续读写做准备；user 分区缺省允许自动格式化。
esp_err_t config_manager_init(void)
{
    esp_err_t err = ESP_OK;

    // 挂载 system 分区（不允许自动格式化）
    if (!fsu_is_storage_mounted())
    {
        err = fsu_mount_storage(false);
        if (err != ESP_OK)
        {
            LOG_ERRORF("Failed to mount system storage: %s", esp_err_to_name(err));
            return err; // 返回具体的错误码，而不是 ESP_FAIL
        }
    }
    LOG_INFO("system storage mounted.");

    // 挂载 user 分区（允许自动格式化）
    if (!fsu_is_user_mounted())
    {
        err = fsu_mount_user(true); // true 表示如果挂载失败则自动格式化
        if (err != ESP_OK)
        {
            LOG_ERRORF("Failed to mount user storage: %s", esp_err_to_name(err));
            return err; // 返回具体的错误码，而不是 ESP_FAIL
        }
    }
    LOG_INFO("user storage mounted.");
    return ESP_OK;
}

// 将 JSON 字段映射到配置结构体。
static void apply_json_to_config(user_config_t *cfg, const cJSON *root)
{
    if (!cfg || !root)
    {
        return;
    }

    const cJSON *item = NULL;

    item = cJSON_GetObjectItemCaseSensitive(root, "deviceId");
    if (cJSON_IsString(item))
    {
        safe_copy(cfg->device_id, sizeof(cfg->device_id), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "deviceName");
    if (cJSON_IsString(item))
    {
        safe_copy(cfg->device_name, sizeof(cfg->device_name), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "host");
    if (cJSON_IsString(item))
    {
        safe_copy(cfg->host, sizeof(cfg->host), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "deviceType");
    if (cJSON_IsString(item))
    {
        // device_type 现在是字符串类型
        safe_copy(cfg->device_type, sizeof(cfg->device_type), item->valuestring);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "motorType");
    if (cJSON_IsNumber(item))
    {
        cfg->motor_type = (int8_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "rpm");
    if (cJSON_IsNumber(item))
    {
        cfg->rpm = (int32_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "patrol");
    if (cJSON_IsNumber(item))
    {
        cfg->patrol = (int16_t)item->valueint;
    }


    item = cJSON_GetObjectItemCaseSensitive(root, "diagnosis");
    if (cJSON_IsNumber(item))
    {
        cfg->diagnosis = (int16_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "report");
    if (cJSON_IsNumber(item))
    {
        cfg->report = (int32_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "months");
    if (cJSON_IsNumber(item))
    {
        cfg->months = (int16_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "battery");
    if (cJSON_IsNumber(item))
    {
        cfg->battery = (int16_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "network");
    if (cJSON_IsNumber(item))
    {
        cfg->network = (int8_t)item->valueint;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "ble");
    if (cJSON_IsBool(item))
    {
        cfg->ble = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "configured");
    if (cJSON_IsBool(item))
    {
        cfg->is_configured = cJSON_IsTrue(item);
    }

    const cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
    if (cJSON_IsObject(wifi))
    {
        const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(wifi, "ssid");
        if (cJSON_IsString(ssid))
        {
            safe_copy(cfg->wifi.ssid, sizeof(cfg->wifi.ssid), ssid->valuestring);
        }
        const cJSON *pass = cJSON_GetObjectItemCaseSensitive(wifi, "pass");
        if (cJSON_IsString(pass))
        {
            safe_copy(cfg->wifi.pass, sizeof(cfg->wifi.pass), pass->valuestring);
        }
    }

    const cJSON *iso = cJSON_GetObjectItemCaseSensitive(root, "iso");
    if (cJSON_IsObject(iso))
    {
        item = cJSON_GetObjectItemCaseSensitive(iso, "standard");
        if (cJSON_IsNumber(item))
        {
            cfg->iso.standard = (int8_t)item->valueint;
        }
        item = cJSON_GetObjectItemCaseSensitive(iso, "category");
        if (cJSON_IsNumber(item))
        {
            cfg->iso.category = (int8_t)item->valueint;
        }

        item = cJSON_GetObjectItemCaseSensitive(iso, "foundation");
        if (cJSON_IsNumber(item))
        {
            cfg->iso.foundation = (int8_t)item->valueint;
        }
    }
}

static void parser_apply_wrapper(cJSON *root, void *ctx)
{
    apply_json_to_config((user_config_t *)ctx, root);
}

// 从路径读取 JSON 并套用到配置。
static esp_err_t load_and_apply(const char *path, user_config_t *cfg)
{
    return fsu_parse_json(path, parser_apply_wrapper, cfg);
}

// 打印指定路径的 JSON 内容（分段避免日志过长）。
static esp_err_t log_config_json(const char *path, const char *label)
{
    if (!path)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (config_manager_init() != ESP_OK)
    {
        return ESP_FAIL;
    }

    size_t len = 0;
    char *json = fsu_read_file_alloc(path, &len);
    if (!json)
    {
        LOG_ERRORF("%s config not found: %s", label ? label : "unknown", path);
        return ESP_ERR_NOT_FOUND;
    }

    LOG_INFOF("%s config (len=%u):", label ? label : "config", (unsigned)len);
    for (size_t i = 0; i < len; i += CONFIG_LOG_CHUNK)
    {
        size_t chunk = (len - i > CONFIG_LOG_CHUNK) ? CONFIG_LOG_CHUNK : (len - i);
        LOG_INFOF("%.*s", (int)chunk, json + i);
    }

    free(json);
    return ESP_OK;
}

// 打印默认配置文件内容，便于调试查看实际默认值。
esp_err_t config_manager_log_default_json(void)
{
    return log_config_json(FILE_PATH_CONFIG_DEFAULT, "Default");
}

esp_err_t config_manager_load(user_config_t *out_cfg)
{
    if (!out_cfg)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 确保所需分区已挂载
    if (config_manager_init() != ESP_OK)
    {
        return ESP_FAIL;
    }

    // 先清零输出结构
    memset(out_cfg, 0, sizeof(*out_cfg));

    // 第一步：加载默认配置（system 分区）
    esp_err_t err = load_and_apply(FILE_PATH_CONFIG_DEFAULT, out_cfg);
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to load default config: %s", FILE_PATH_CONFIG_DEFAULT);
        return err;
    }

    // 第二步：若存在用户配置，则覆盖默认配置
    if (fsu_is_user_mounted() && fsu_file_exists(FILE_PATH_CONFIG_USER))
    {
        err = load_and_apply(FILE_PATH_CONFIG_USER, out_cfg);
        if (err != ESP_OK)
        {
            LOG_WARN("User config parse failed; using defaults");
            (void)config_manager_log_default_json();
            out_cfg->is_configured = false;
            g_user_config = *out_cfg;
            return ESP_OK;
        }
        // 仅在用户配置解析成功时打印当前用户配置
        (void)log_config_json(FILE_PATH_CONFIG_USER, "User");
        g_user_config = *out_cfg;
        return ESP_OK; // 用户配置成功覆盖
    }

    // 没有用户配置，保持默认值并标记未配置
    LOG_INFO("User config not found; using defaults");
    (void)config_manager_log_default_json();
    out_cfg->is_configured = false;
    g_user_config = *out_cfg;
    return ESP_OK;
}

// 直接保存前端传入的原始 JSON。
esp_err_t config_manager_save_user_json(const char *json)
{
    if (!json)
    {
        return ESP_ERR_INVALID_ARG;
    }
    LOG_DEBUGF("file contents: %s", json);

    if (config_manager_init() != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (!fsu_is_user_mounted())
    {
        LOG_ERROR("User SPIFFS not mounted");
        return ESP_FAIL;
    }
    LOG_DEBUGF("saving user config to %s", FILE_PATH_CONFIG_USER);
    return fsu_write_file(FILE_PATH_CONFIG_USER, json, strlen(json));
}

// 将结构体转 JSON 后写入用户分区。
esp_err_t config_manager_save_user(const user_config_t *cfg)
{
    if (!cfg)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "deviceId", cfg->device_id);
    cJSON_AddStringToObject(root, "deviceName", cfg->device_name);
    cJSON_AddStringToObject(root, "deviceType", cfg->device_type);
    cJSON_AddNumberToObject(root, "motorType", cfg->motor_type);
    cJSON_AddNumberToObject(root, "months", cfg->months);
    cJSON_AddStringToObject(root, "host", cfg->host);
    cJSON_AddNumberToObject(root, "patrol", cfg->patrol);
    cJSON_AddNumberToObject(root, "diagnosis", cfg->diagnosis);
    cJSON_AddNumberToObject(root, "report", cfg->report);
    cJSON_AddNumberToObject(root, "battery", cfg->battery);
    cJSON_AddNumberToObject(root, "rpm", cfg->rpm);
    cJSON_AddNumberToObject(root, "network", cfg->network);
    cJSON_AddBoolToObject(root, "ble", cfg->ble);
    cJSON_AddBoolToObject(root, "configured", cfg->is_configured);

    cJSON *iso = cJSON_CreateObject();
    cJSON_AddNumberToObject(iso, "standard", cfg->iso.standard);
    cJSON_AddNumberToObject(iso, "category", cfg->iso.category);
    cJSON_AddNumberToObject(iso, "foundation", cfg->iso.foundation);
    cJSON_AddItemToObject(root, "iso", iso);

    cJSON *wifi = cJSON_CreateObject();
    cJSON_AddStringToObject(wifi, "ssid", cfg->wifi.ssid);
    cJSON_AddStringToObject(wifi, "pass", cfg->wifi.pass);
    cJSON_AddItemToObject(root, "wifi", wifi);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_str)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = config_manager_save_user_json(json_str);
    free(json_str);
    return err;
}
