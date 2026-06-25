#include "report_upload_cache.h"

#include "fs_utils.h"
#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REPORT_UPLOAD_CACHE_MAX_FILES 1024
#define REPORT_UPLOAD_CACHE_PATH_LEN 48
#define REPORT_UPLOAD_CACHE_PREFIX "/user/report_retry_"

static void cache_path_for_slot(size_t slot, char *path, size_t path_len)
{
    snprintf(path, path_len, REPORT_UPLOAD_CACHE_PREFIX "%u.json", (unsigned)slot);
}

static esp_err_t ensure_user_storage(void)
{
    if (fsu_is_user_mounted()) {
        return ESP_OK;
    }
    return fsu_mount_user(true);
}

esp_err_t report_upload_cache_save_failed(const char *json)
{
    if (!json) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ensure_user_storage();
    if (err != ESP_OK) {
        LOG_ERRORF("Cannot save failed report: user storage unavailable: %s", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < REPORT_UPLOAD_CACHE_MAX_FILES; ++i) {
        char path[REPORT_UPLOAD_CACHE_PATH_LEN];
        cache_path_for_slot(i, path, sizeof(path));
        if (i % 16 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (fsu_file_exists(path)) {
            continue;
        }

        err = fsu_write_file(path, json, strlen(json));
        if (err == ESP_OK) {
            LOG_WARNF("Saved failed report JSON for retry: %s", path);
        } else {
            LOG_ERRORF("Failed to save report retry file %s: %s", path, esp_err_to_name(err));
        }
        return err;
    }

    LOG_ERRORF("Report retry cache full; cannot save failed report (max=%u)",
               (unsigned)REPORT_UPLOAD_CACHE_MAX_FILES);
    return ESP_ERR_NO_MEM;
}

esp_err_t report_upload_cache_flush(report_upload_cache_sender_t sender, void *ctx)
{
    if (!sender) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ensure_user_storage();
    if (err != ESP_OK) {
        LOG_WARNF("Cannot flush report retry cache: user storage unavailable: %s", esp_err_to_name(err));
        return err;
    }

    esp_err_t first_err = ESP_OK;

    DIR *dir = opendir("/user");
    if (!dir) {
        LOG_WARNF("Cannot open /user directory for cache flush");
        return ESP_FAIL;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        // We are looking for files like "report_retry_X.json"
        if (strncmp(ent->d_name, "report_retry_", 13) != 0) {
            continue;
        }

        char path[300];
        snprintf(path, sizeof(path), "/user/%s", ent->d_name);

        size_t len = 0;
        char *json = fsu_read_file_alloc(path, &len);
        if (!json || len == 0) {
            LOG_WARNF("Removing unreadable or empty report retry file: %s", path);
            free(json);
            (void)unlink(path);
            continue;
        }

        LOG_INFOF("Retrying cached report upload: %s", path);
        err = sender(json, ctx);
        free(json);
        if (err == ESP_OK) {
            if (unlink(path) == 0) {
                LOG_INFOF("Removed uploaded report retry file: %s", path);
            } else {
                LOG_WARNF("Uploaded cached report but failed to remove retry file: %s", path);
                if (first_err == ESP_OK) {
                    first_err = ESP_FAIL;
                }
            }
        } else {
            LOG_WARNF("Cached report upload still failed: %s (%s)", path, esp_err_to_name(err));
            if (first_err == ESP_OK) {
                first_err = err;
            }
        }
    }

    closedir(dir);
    return first_err;
}
