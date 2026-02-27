#include "fs_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "logger.h"

static bool s_storage_mounted = false;
static bool s_user_mounted = false;

static esp_err_t mount_spiffs(const char *label, const char *base_path, int max_files, bool format_if_mount_failed)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = label,
        .max_files = max_files,
        .format_if_mount_failed = format_if_mount_failed,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK; // already mounted
    }
    if (err != ESP_OK) {
        LOG_ERRORF("Failed to mount SPIFFS '%s' at '%s': %s", label, base_path, esp_err_to_name(err));
        return err;
    }

    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(label, &total, &used) == ESP_OK) {
        LOG_INFOF("SPIFFS '%s' mounted at '%s' (used %u / total %u)",
                  label, base_path, (unsigned)used, (unsigned)total);
    }
    return ESP_OK;
}

esp_err_t fsu_mount_storage(bool format_if_mount_failed)
{
    esp_err_t err = mount_spiffs("system", "/system", 10, format_if_mount_failed);
    if (err == ESP_OK) {
        s_storage_mounted = true;
    }
    return err;
}

esp_err_t fsu_mount_user(bool format_if_mount_failed)
{
    esp_err_t err = mount_spiffs("user", "/user", 5, format_if_mount_failed);
    if (err == ESP_OK) {
        s_user_mounted = true;
    }
    return err;
}

bool fsu_is_storage_mounted(void) { return s_storage_mounted; }
bool fsu_is_user_mounted(void) { return s_user_mounted; }

bool fsu_file_exists(const char *path)
{
    struct stat st;
    return (path && stat(path, &st) == 0 && S_ISREG(st.st_mode));
}

char *fsu_read_file_alloc(const char *path, size_t *out_len)
{
    if (!path) {
        return NULL;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long size = ftell(f);
    if (size < 0) {
        fclose(f);
        return NULL;
    }
    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return NULL;
    }

    char *buf = (char *)calloc((size_t)size + 1, 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }

    size_t read_sz = fread(buf, 1, (size_t)size, f);
    fclose(f);
    buf[read_sz] = '\0';
    if (out_len) {
        *out_len = read_sz;
    }
    return buf;
}

esp_err_t fsu_write_file(const char *path, const char *data, size_t len)
{
    if (!path || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        LOG_ERRORF("Failed to open %s for writing", path);
        return ESP_FAIL;
    }
    size_t written = fwrite(data, 1, len, f);
    fclose(f);
    if (written != len) {
        LOG_ERRORF("Write failed (%u/%u)", (unsigned)written, (unsigned)len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t fsu_parse_json(const char *path, fsu_json_parser_t parser, void *ctx)
{
    if (!parser || !path) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t len = 0;
    char *json = fsu_read_file_alloc(path, &len);
    if (!json) {
        return ESP_ERR_NOT_FOUND;
    }
    cJSON *root = cJSON_Parse(json);
    free(json);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    parser(root, ctx);
    cJSON_Delete(root);
    return ESP_OK;
}
