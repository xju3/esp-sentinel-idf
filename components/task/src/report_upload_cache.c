#include "report_upload_cache.h"

#include "cJSON.h"
#include "fs_utils.h"
#include "logger.h"

#include "esp_rtc_time.h"
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
  uint64_t min_time_s;
  uint64_t max_time_s;
} report_cache_index_t;

static bool parse_timestamp_filename(const char *name, uint64_t *out_time_s) {
  if (!name || !out_time_s) {
    return false;
  }

  while (*name == '/') {
    ++name;
  }

  uint64_t value = 0;
  size_t digits = 0;
  while (name[digits] >= '0' && name[digits] <= '9') {
    value = value * 10U + (uint64_t)(name[digits] - '0');
    ++digits;
  }

  if (digits == 0 || value == 0 || strcmp(name + digits, ".json") != 0) {
    return false;
  }

  *out_time_s = value;
  return true;
}

static esp_err_t scan_cache(report_cache_index_t *index) {
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
    uint64_t time_s = 0;
    if (!parse_timestamp_filename(entry->d_name, &time_s)) {
      continue;
    }

    if (index->count == 0 || time_s < index->min_time_s) {
      index->min_time_s = time_s;
    }
    if (index->count == 0 || time_s > index->max_time_s) {
      index->max_time_s = time_s;
    }
    ++index->count;
  }

  closedir(dir);
  return ESP_OK;
}

static esp_err_t json_with_delay(const char *json, uint32_t delay_s,
                                 char **out_json) {
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
  cJSON_DeleteItemFromObjectCaseSensitive(root, "delay");
  if (!cJSON_AddNumberToObject(root, "delay", delay_s)) {
    cJSON_Delete(root);
    return ESP_ERR_NO_MEM;
  }

  *out_json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  return *out_json ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t json_without_delay(const char *json, char **out_json) {
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
  cJSON_DeleteItemFromObjectCaseSensitive(root, "delay");
  *out_json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  return *out_json ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t report_upload_cache_store(const char *json, uint64_t *out_time_s) {
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
  if (err != ESP_OK) {
    return err;
  }

  uint64_t time_s = esp_rtc_get_time_us() / 1000000ULL;
  if (index.count > 0 && time_s <= index.max_time_s) {
    time_s = index.max_time_s + 1;
  }

  char *stored_json = NULL;
  err = json_without_delay(json, &stored_json);
  if (err != ESP_OK) {
    return err;
  }

  char final_path[48];
  snprintf(final_path, sizeof(final_path), REPORT_CACHE_DIR "/%llu.json",
           (unsigned long long)time_s);

  (void)unlink(REPORT_CACHE_TEMP_PATH);
  err =
      fsu_write_file(REPORT_CACHE_TEMP_PATH, stored_json, strlen(stored_json));
  free(stored_json);
  if (err != ESP_OK) {
    return err;
  }

  if (rename(REPORT_CACHE_TEMP_PATH, final_path) != 0) {
    (void)unlink(REPORT_CACHE_TEMP_PATH);
    LOG_ERRORF("Failed to finalize cached report: time_s=%llu",
               (unsigned long long)time_s);
    return ESP_FAIL;
  }

  if (out_time_s) {
    *out_time_s = time_s;
  }
  LOG_INFOF("Report persisted for retry: time_s=%llu file=%s",
            (unsigned long long)time_s, final_path);
  return ESP_OK;
}

esp_err_t report_upload_cache_flush(report_upload_cache_sender_t sender,
                                    void *ctx) {
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
    snprintf(path, sizeof(path), REPORT_CACHE_DIR "/%llu.json",
             (unsigned long long)index.min_time_s);
    char *stored_json = fsu_read_file_alloc(path, NULL);
    if (!stored_json) {
      LOG_WARNF("Failed to read cached report: time_s=%llu",
                (unsigned long long)index.min_time_s);
      return ESP_FAIL;
    }

    if (index.count > UINT32_MAX) {
      free(stored_json);
      return ESP_ERR_INVALID_SIZE;
    }

    uint64_t now_s = esp_rtc_get_time_us() / 1000000ULL;
    uint32_t delay_s =
        (now_s > index.min_time_s) ? (uint32_t)(now_s - index.min_time_s) : 0;

    char *upload_json = NULL;
    err = json_with_delay(stored_json, delay_s, &upload_json);
    free(stored_json);
    if (err != ESP_OK) {
      return err;
    }

    err = sender(upload_json, ctx);
    free(upload_json);
    if (err != ESP_OK) {
      LOG_WARNF(
          "Cached report upload deferred: time_s=%llu delay_s=%lu error=%s",
          (unsigned long long)index.min_time_s, (unsigned long)delay_s,
          esp_err_to_name(err));
      return err;
    }

    if (unlink(path) != 0) {
      LOG_WARNF("Cached report uploaded but file removal failed: time_s=%llu "
                "delay_s=%lu",
                (unsigned long long)index.min_time_s, (unsigned long)delay_s);
      return ESP_FAIL;
    }
    LOG_INFOF("Cached report uploaded: time_s=%llu delay_s=%lu",
              (unsigned long long)index.min_time_s, (unsigned long)delay_s);
  }
}
