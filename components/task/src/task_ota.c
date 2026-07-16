#include "task_ota.h"
#include "config_manager.h"
#include "logger.h"
#include "system_lock.h"


#include "http_proxy.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "drv_4g.h"

#include <stdlib.h>
#include <string.h>

extern esp_err_t bsp_4g_ota_download_and_write(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle);

#define OTA_STATE_NAMESPACE "ota_state"
#define OTA_STATE_KEY_TASK_ID "task_id"
#define OTA_STATE_KEY_RESULT "result"
#define OTA_STATE_KEY_TARGET_ADDR "target_addr"

// 内部方法：上报 OTA 结果 (HTTP PUT)
static esp_err_t report_ota_result(const char *task_id, int result_code)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s", g_user_config.api_host, task_id);
    
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"result\": %d}", result_code);
    
    LOG_INFOF("Reporting OTA result %d to %s", result_code, url);

    esp_err_t err = bsp_4g_http_put(url, payload);
    if (err == ESP_OK) {
        LOG_INFO("4G HTTP PUT result reported successfully.");
    } else {
        LOG_ERROR("4G HTTP PUT result report failed.");
    }
    return err;
}

// 内部方法：上报 OTA 最终完成结果 (HTTP POST /complete)
static esp_err_t report_ota_complete(const char *task_id, int result_code)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/%s/complete/%d", g_user_config.api_host, task_id, result_code);
    
    LOG_INFOF("Reporting OTA complete result %d to %s", result_code, url);

    char *response = NULL;
    esp_err_t err = bsp_4g_http_post_json(url, "", &response);
    if (err == ESP_OK) {
        LOG_INFO("4G HTTP POST complete result reported successfully.");
    } else {
        LOG_ERROR("4G HTTP POST complete result report failed.");
    }
    free(response);
    return err;
}

static esp_err_t fetch_ota_download_parameters(const char *task_id,
                                               int *out_size,
                                               char **out_access_key)
{
    if (!task_id || task_id[0] == '\0' || !out_size || !out_access_key) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_size = 0;
    *out_access_key = NULL;

    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s",
             g_user_config.api_host, task_id);

    char *json_response = NULL;
    esp_err_t err = http_proxy_get(url, &json_response);
    if (err != ESP_OK || !json_response) {
        free(json_response);
        return err != ESP_OK ? err : ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *root = cJSON_Parse(json_response);
    free(json_response);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const cJSON *size_item = cJSON_GetObjectItemCaseSensitive(root, "size");
    const cJSON *key_item = cJSON_GetObjectItemCaseSensitive(root, "access_key");
    if (!cJSON_IsNumber(size_item) || size_item->valueint <= 0) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (cJSON_IsString(key_item) && key_item->valuestring &&
        key_item->valuestring[0] != '\0') {
        size_t key_len = strlen(key_item->valuestring);
        *out_access_key = malloc(key_len + 1);
        if (!*out_access_key) {
            cJSON_Delete(root);
            return ESP_ERR_NO_MEM;
        }
        memcpy(*out_access_key, key_item->valuestring, key_len + 1);
    }

    *out_size = size_item->valueint;
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t persist_pending_ota(const char *task_id,
                                     uint32_t target_address)
{
    nvs_handle_t handle;
    if (!task_id || task_id[0] == '\0' || target_address == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_open(OTA_STATE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, OTA_STATE_KEY_TASK_ID, task_id);
    if (err == ESP_OK) {
        err = nvs_set_u32(handle, OTA_STATE_KEY_TARGET_ADDR, target_address);
    }
    if (err == ESP_OK) {
        esp_err_t erase_err = nvs_erase_key(handle, OTA_STATE_KEY_RESULT);
        if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
            err = erase_err;
        }
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t task_ota_finalize_boot_status(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OTA_STATE_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = 0;
    err = nvs_get_str(handle, OTA_STATE_KEY_TASK_ID, NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return ESP_OK;
    }
    if (err != ESP_OK || required_size <= 1) {
        nvs_close(handle);
        return err != ESP_OK ? err : ESP_ERR_INVALID_STATE;
    }

    int8_t stored_result = 0;
    if (nvs_get_i8(handle, OTA_STATE_KEY_RESULT, &stored_result) == ESP_OK) {
        nvs_close(handle);
        return ESP_OK;
    }

    uint32_t target_address = 0;
    err = nvs_get_u32(handle, OTA_STATE_KEY_TARGET_ADDR, &target_address);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    int8_t result_code = 1;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running && running->address == target_address) {
        esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;
        err = esp_ota_get_state_partition(running, &ota_state);
        if (err == ESP_OK && ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            LOG_INFO("OTA image booted successfully. Marking app valid locally.");
            err = esp_ota_mark_app_valid_cancel_rollback();
            if (err == ESP_OK) {
                result_code = 0;
            } else {
                LOG_ERRORF("Failed to mark OTA image valid: %s",
                           esp_err_to_name(err));
            }
        } else if (err == ESP_OK && ota_state == ESP_OTA_IMG_VALID) {
            // Handles a reset after the image was marked valid but before the
            // completion result was committed to NVS.
            result_code = 0;
        } else {
            LOG_WARNF("Unexpected OTA image state: %d", (int)ota_state);
        }
    } else {
        LOG_WARNF("OTA target did not boot or was rolled back: target=0x%lx running=0x%lx",
                  (unsigned long)target_address,
                  running ? (unsigned long)running->address : 0UL);
    }

    esp_err_t store_err = nvs_set_i8(handle, OTA_STATE_KEY_RESULT, result_code);
    if (store_err == ESP_OK) {
        store_err = nvs_commit(handle);
    }
    nvs_close(handle);
    return store_err;
}

bool task_ota_completion_pending_for(const char *task_id)
{
    if (!task_id || task_id[0] == '\0') {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(OTA_STATE_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t required_size = 0;
    esp_err_t err = nvs_get_str(handle, OTA_STATE_KEY_TASK_ID, NULL,
                                &required_size);
    if (err != ESP_OK || required_size <= 1) {
        nvs_close(handle);
        return false;
    }

    char *pending_task_id = malloc(required_size);
    if (!pending_task_id) {
        nvs_close(handle);
        return false;
    }

    err = nvs_get_str(handle, OTA_STATE_KEY_TASK_ID, pending_task_id,
                      &required_size);
    const bool pending = err == ESP_OK && strcmp(task_id, pending_task_id) == 0;
    free(pending_task_id);
    nvs_close(handle);
    return pending;
}

void task_ota_report_pending_completion(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OTA_STATE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return;
    }

    size_t required_size = 0;
    err = nvs_get_str(handle, OTA_STATE_KEY_TASK_ID, NULL, &required_size);
    if (err != ESP_OK || required_size <= 1) {
        nvs_close(handle);
        return;
    }

    int8_t result_code = 1;
    if (nvs_get_i8(handle, OTA_STATE_KEY_RESULT, &result_code) != ESP_OK) {
        LOG_WARN("OTA completion is pending local boot finalization.");
        nvs_close(handle);
        return;
    }

    char *task_id = malloc(required_size);
    if (!task_id) {
        nvs_close(handle);
        return;
    }
    err = nvs_get_str(handle, OTA_STATE_KEY_TASK_ID, task_id, &required_size);
    if (err == ESP_OK && report_ota_complete(task_id, result_code) == ESP_OK) {
        (void)nvs_erase_key(handle, OTA_STATE_KEY_TASK_ID);
        (void)nvs_erase_key(handle, OTA_STATE_KEY_RESULT);
        (void)nvs_erase_key(handle, OTA_STATE_KEY_TARGET_ADDR);
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            LOG_WARNF("Failed to clear reported OTA state: %s",
                      esp_err_to_name(err));
        }
    } else {
        LOG_WARN("Keeping pending OTA completion for a later successful report.");
    }

    free(task_id);
    nvs_close(handle);
}

static esp_err_t perform_ota_download_url(const char *fw_url, int fw_size,
                                          const char *access_key,
                                          uint32_t *out_partition_address)
{
    if (out_partition_address) {
        *out_partition_address = 0;
    }
    if (!fw_url || fw_url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ota_err = ESP_FAIL;

    if (fw_size <= 0) {
        LOG_ERROR("4G OTA from direct URL requires known firmware size.");
        return ESP_ERR_INVALID_ARG;
    }

    char *clean_url = strdup(fw_url);
    if (!clean_url) {
        return ESP_ERR_NO_MEM;
    }

    char *ver_ptr = strstr(clean_url, "?ver=");
    if (!ver_ptr) {
        ver_ptr = strstr(clean_url, "&ver=");
    }
    
    if (ver_ptr) {
        char *end_ptr = strchr(ver_ptr + 1, '&');
        if (end_ptr) {
            if (ver_ptr[0] == '?') {
                *end_ptr = '?';
                memmove(ver_ptr, end_ptr, strlen(end_ptr) + 1);
            } else {
                memmove(ver_ptr, end_ptr, strlen(end_ptr) + 1);
            }
        } else {
            *ver_ptr = '\0';
        }
    }

    LOG_INFOF("Starting OTA chunked download via 4G AT Mode, URL: %s", clean_url);
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition) {
        LOG_INFOF("Writing to partition subtype %d at offset 0x%lx", update_partition->subtype, update_partition->address);
        esp_ota_handle_t update_handle = 0;

        if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) == ESP_OK) {
            if (bsp_4g_ota_download_and_write(clean_url, fw_size, access_key, update_handle) == ESP_OK) {
                if (esp_ota_end(update_handle) == ESP_OK) {
                    if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                        ota_err = ESP_OK;
                        if (out_partition_address) {
                            *out_partition_address = update_partition->address;
                        }
                        LOG_INFO("OTA Success! Boot partition configured.");
                    }
                }
            } else {
                LOG_ERROR("4G OTA Download aborted due to error.");
                esp_ota_abort(update_handle);
            }
        }
    }

    free(clean_url);
    return ota_err;
}

void execute_ota_update_sync(const char *task_id)
{
    LOG_INFOF("=== Starting OTA Update Process for Task: %s ===", task_id);

    // 1. 停止触发新任务，准备等待流水线排空
    LOG_INFO("1. Waiting for pipeline drain (skipped, synchronous scheduling in use).");

    // 2. 加锁独占系统，屏蔽硬件扫描被以外唤醒
    LOG_INFO("2. Locking system task mutex for exclusive OTA operations...");
    lock_system_task();

    // 3. 向服务器获取固件元信息
    char url[256];
    char *json_response = NULL;
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s", g_user_config.api_host, task_id);
    
    LOG_INFOF("Fetching OTA info from: %s", url);
    if (http_proxy_get(url, &json_response) != ESP_OK || json_response == NULL) {
        LOG_ERROR("Failed to fetch OTA info.");
        report_ota_result(task_id, 1);
        unlock_system_task();

        return;
    }

    cJSON *root = cJSON_Parse(json_response);
    free(json_response);

    if (!root) {
        LOG_ERROR("Failed to parse OTA info JSON.");
        report_ota_result(task_id, 2);
        unlock_system_task();

        return;
    }

    cJSON *size_item = cJSON_GetObjectItem(root, "size");
    cJSON *path_item = cJSON_GetObjectItem(root, "path");
    cJSON *key_item = cJSON_GetObjectItem(root, "access_key");

    if (!cJSON_IsNumber(size_item) || !cJSON_IsString(path_item)) {
        LOG_ERROR("Invalid OTA info format (missing size or path).");
        cJSON_Delete(root);
        report_ota_result(task_id, 3);
        unlock_system_task();

        return;
    }

    int fw_size = size_item->valueint;
    const char *fw_url = path_item->valuestring;
    const char *access_key = cJSON_IsString(key_item) ? key_item->valuestring : "";
    LOG_INFOF("OTA Firmware Info: size=%d, url=%s", fw_size, fw_url);

    // 4. 执行下载与写分区
    uint32_t target_address = 0;
    esp_err_t ota_err = perform_ota_download_url(fw_url, fw_size, access_key,
                                                 &target_address);

    cJSON_Delete(root);

    // 5. 汇报结果并根据情况重启系统
    if (ota_err == ESP_OK) {
        esp_err_t state_err = persist_pending_ota(task_id, target_address);
        if (state_err != ESP_OK) {
            LOG_ERRORF("Failed to persist OTA completion marker: %s",
                       esp_err_to_name(state_err));
            const esp_partition_t *running = esp_ota_get_running_partition();
            if (running) {
                (void)esp_ota_set_boot_partition(running);
            }
            (void)report_ota_result(task_id, 4);
            unlock_system_task();
            return;
        }
        LOG_INFO("OTA Update completed successfully. System will restart in 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart(); // 重启应用新固件，如果失败底层回滚机制会自动切换回原分区
    } else {
        LOG_ERROR("OTA Update failed.");
        report_ota_result(task_id, 4); // 大于 0 表示异常
        unlock_system_task();          // 释放排他锁

    }
}

void execute_ota_update_from_url_sync(const char *task_id, const char *fw_url)
{
    LOG_INFOF("=== Starting OTA Update from URL for Task: %s ===", task_id ? task_id : "");
    if (!fw_url || fw_url[0] == '\0') {
        LOG_ERROR("OTA URL is empty.");
        report_ota_result(task_id ? task_id : "", 3);
        return;
    }

    LOG_INFO("1. Pausing periodic DAQ tasks and waiting for pipeline drain...");


    LOG_INFO("2. Locking system task mutex for exclusive OTA operations...");
    lock_system_task();

    LOG_INFOF("OTA direct firmware URL: %s", fw_url);
    int fw_size = 0;
    char *access_key = NULL;
    esp_err_t metadata_err =
        fetch_ota_download_parameters(task_id, &fw_size, &access_key);
    if (metadata_err != ESP_OK) {
        LOG_ERRORF("Failed to fetch OTA download parameters: %s",
                   esp_err_to_name(metadata_err));
        (void)report_ota_complete(task_id ? task_id : "", 1);
        unlock_system_task();
        return;
    }

    uint32_t target_address = 0;
    esp_err_t ota_err = perform_ota_download_url(fw_url, fw_size,
                                                 access_key ? access_key : "",
                                                 &target_address);
    free(access_key);

    if (ota_err == ESP_OK) {
        esp_err_t state_err = persist_pending_ota(task_id, target_address);
        if (state_err != ESP_OK) {
            LOG_ERRORF("Failed to persist OTA completion marker: %s",
                       esp_err_to_name(state_err));
            const esp_partition_t *running = esp_ota_get_running_partition();
            if (running) {
                (void)esp_ota_set_boot_partition(running);
            }
            (void)report_ota_complete(task_id ? task_id : "", 1);
            unlock_system_task();
            return;
        }
        LOG_INFO("OTA Update downloaded successfully. System will restart in 3 seconds to verify...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    } else {
        LOG_ERROR("OTA Update failed during download.");
        report_ota_complete(task_id ? task_id : "", 1);
        unlock_system_task();

    }
}
