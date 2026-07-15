#include "report_upload_cache.h"

#include "cJSON.h"
#include "fs_utils.h"
#include "logger.h"

#include <dirent.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define REPORT_CACHE_DIR "/user"
#define REPORT_CACHE_TEMP_PATH REPORT_CACHE_DIR "/.report.tmp"

typedef struct {
    size_t count;
    uint32_t min_seq;
    uint32_t max_seq;
} report_cache_index_t;

static bool parse_sequence_filename(const char *name, uint32_t *out_seq)
{
    if (!name || !out_seq) {
        return false;
    }

    while (*name == '/') {
        ++name;
    }

    uint64_t value = 0;
    size_t digits = 0;
    while (name[digits] >= '0' && name[digits] <= '9') {
        value = value * 10U + (uint32_t)(name[digits] - '0');
        if (value > UINT32_MAX) {
            return false;
        }
        ++digits;
    }

    if (digits == 0 || value == 0 || strcmp(name + digits, ".json") != 0) {
        return false;
    }

    *out_seq = (uint32_t)value;
    return true;
}

static esp_err_t scan_cache(report_cache_index_t *index)
{
    if (!index) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(index, 0, sizeof(*index));
    DIR *dir = opendir(REPORT_CACHE_DIR);
    if (!dir) {
        LOG_ERROR("Failed to open report cache directory");
        return ESP_FAIL;
    }

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t seq = 0;
        if (!parse_sequence_filename(entry->d_name, &seq)) {
            continue;
        }

        if (index->count == 0 || seq < index->min_seq) {
            index->min_seq = seq;
        }
        if (index->count == 0 || seq > index->max_seq) {
            index->max_seq = seq;
        }
        ++index->count;
    }

    closedir(dir);
    return ESP_OK;
}

static esp_err_t json_with_sequence(const char *json, uint32_t seq,
                                    char **out_json)
{
    if (!json || !out_json) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_json = NULL;

    cJSON *root = cJSON_Parse(json);
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(root, "seq");
    if (!cJSON_AddNumberToObject(root, "seq", seq)) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    *out_json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return *out_json ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t json_without_sequence(const char *json, char **out_json)
{
    if (!json || !out_json) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_json = NULL;

    cJSON *root = cJSON_Parse(json);
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(root, "seq");
    *out_json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return *out_json ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t report_upload_cache_store(const char *json, uint32_t *out_seq)
{
    if (!json) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!fsu_is_user_mounted()) {
        LOG_ERROR("Cannot persist report: user storage is not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    report_cache_index_t index = {0};
    esp_err_t err = scan_cache(&index);
    if (err != ESP_OK) {
        return err;
    }
    if (index.count > 0 && index.max_seq == UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t seq = index.count == 0 ? 1U : index.max_seq + 1U;
    // The report's final seq depends on how many older files exist when a
    // future current report succeeds, so do not freeze seq at storage time.
    char *stored_json = NULL;
    err = json_without_sequence(json, &stored_json);
    if (err != ESP_OK) {
        return err;
    }

    char final_path[48];
    snprintf(final_path, sizeof(final_path), REPORT_CACHE_DIR "/%lu.json",
             (unsigned long)seq);

    (void)unlink(REPORT_CACHE_TEMP_PATH);
    err = fsu_write_file(REPORT_CACHE_TEMP_PATH, stored_json,
                         strlen(stored_json));
    free(stored_json);
    if (err != ESP_OK) {
        return err;
    }

    if (rename(REPORT_CACHE_TEMP_PATH, final_path) != 0) {
        (void)unlink(REPORT_CACHE_TEMP_PATH);
        LOG_ERRORF("Failed to finalize cached report: seq=%lu",
                   (unsigned long)seq);
        return ESP_FAIL;
    }

    if (out_seq) {
        *out_seq = seq;
    }
    LOG_INFOF("Report persisted for retry: file_no=%lu file=%s",
              (unsigned long)seq, final_path);
    return ESP_OK;
}

esp_err_t report_upload_cache_flush(report_upload_cache_sender_t sender,
                                    void *ctx)
{
    if (!sender) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!fsu_is_user_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }

    while (true) {
        report_cache_index_t index = {0};
        esp_err_t err = scan_cache(&index);
        if (err != ESP_OK || index.count == 0) {
            return err;
        }

        char path[48];
        snprintf(path, sizeof(path), REPORT_CACHE_DIR "/%lu.json",
                 (unsigned long)index.min_seq);
        char *stored_json = fsu_read_file_alloc(path, NULL);
        if (!stored_json) {
            LOG_WARNF("Failed to read cached report: file_no=%lu",
                      (unsigned long)index.min_seq);
            return ESP_FAIL;
        }

        if (index.count > UINT32_MAX) {
            free(stored_json);
            return ESP_ERR_INVALID_SIZE;
        }

        // With N cached files, the oldest is N periods before the fresh
        // seq=0 report, and the newest cached file is one period before it.
        char *upload_json = NULL;
        err = json_with_sequence(stored_json, (uint32_t)index.count,
                                 &upload_json);
        free(stored_json);
        if (err != ESP_OK) {
            return err;
        }

        err = sender(upload_json, ctx);
        free(upload_json);
        if (err != ESP_OK) {
            LOG_WARNF("Cached report upload deferred: file_no=%lu seq=%lu error=%s",
                      (unsigned long)index.min_seq,
                      (unsigned long)index.count, esp_err_to_name(err));
            return err;
        }

        if (unlink(path) != 0) {
            LOG_WARNF("Cached report uploaded but file removal failed: file_no=%lu seq=%lu",
                      (unsigned long)index.min_seq,
                      (unsigned long)index.count);
            return ESP_FAIL;
        }
        LOG_INFOF("Cached report uploaded: file_no=%lu seq=%lu",
                  (unsigned long)index.min_seq,
                  (unsigned long)index.count);
    }
}
