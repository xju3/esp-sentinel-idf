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

extern esp_err_t bsp_4g_ota_download_and_write(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle);

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

bool task_ota_status_pending(void)
{
    nvs_handle_t handle;
    if (nvs_open("ota_state", NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t required_size = 0;
    const bool pending =
        nvs_get_str(handle, "task_id", NULL, &required_size) == ESP_OK &&
        required_size > 0;
    nvs_close(handle);
    return pending;
}

void check_and_report_ota_status(void)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("ota_state", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    size_t required_size = 0;
    err = nvs_get_str(my_handle, "task_id", NULL, &required_size);
    if (err == ESP_OK && required_size > 0) {
        char *task_id = malloc(required_size);
        if (task_id) {
            nvs_get_str(my_handle, "task_id", task_id, &required_size);
            
            esp_ota_img_states_t ota_state;
            const esp_partition_t *running = esp_ota_get_running_partition();

            int8_t result_code = 1;
            if (nvs_get_i8(my_handle, "result", &result_code) != ESP_OK) {
                result_code = 1;
                if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
                    ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                    LOG_INFO("OTA verification pending. Marking app valid.");
                    esp_err_t valid_err = esp_ota_mark_app_valid_cancel_rollback();
                    if (valid_err == ESP_OK) {
                        result_code = 0;
                    } else {
                        LOG_ERRORF("Failed to mark OTA image valid: %s",
                                   esp_err_to_name(valid_err));
                    }
                } else {
                    LOG_WARN("OTA verification failed or rollback detected.");
                }
                nvs_set_i8(my_handle, "result", result_code);
                nvs_commit(my_handle);
            }

            esp_err_t report_err = report_ota_complete(task_id, result_code);

            if (report_err == ESP_OK) {
                nvs_erase_key(my_handle, "task_id");
                nvs_erase_key(my_handle, "result");
                nvs_commit(my_handle);
            } else {
                LOG_WARN("Keeping pending OTA status for the next wakeup.");
            }
            free(task_id);
        }
    }
    nvs_close(my_handle);
}

static esp_err_t perform_ota_download_url(const char *fw_url, int fw_size, const char *access_key)
{
    if (!fw_url || fw_url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ota_err = ESP_FAIL;

    if (fw_size <= 0) {
        LOG_ERROR("4G OTA from direct URL requires known firmware size.");
        return ESP_ERR_INVALID_ARG;
    }

    LOG_INFO("Starting OTA chunked download via 4G AT Mode...");
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition) {
        LOG_INFOF("Writing to partition subtype %d at offset 0x%lx", update_partition->subtype, update_partition->address);
        esp_ota_handle_t update_handle = 0;

        if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) == ESP_OK) {
            if (bsp_4g_ota_download_and_write(fw_url, fw_size, access_key, update_handle) == ESP_OK) {
                if (esp_ota_end(update_handle) == ESP_OK) {
                    if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                        ota_err = ESP_OK;
                        LOG_INFO("OTA Success! Boot partition configured.");
                    }
                }
            } else {
                LOG_ERROR("4G OTA Download aborted due to error.");
                esp_ota_abort(update_handle);
            }
        }
    }

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
    esp_err_t ota_err = perform_ota_download_url(fw_url, fw_size, access_key);

    cJSON_Delete(root);

    // 5. 汇报结果并根据情况重启系统
    if (ota_err == ESP_OK) {
        report_ota_result(task_id, 0); // 0 表示成功
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
    esp_err_t ota_err = perform_ota_download_url(fw_url, 0, "");

    if (ota_err == ESP_OK) {
        nvs_handle_t my_handle;
        if (nvs_open("ota_state", NVS_READWRITE, &my_handle) == ESP_OK) {
            nvs_set_str(my_handle, "task_id", task_id ? task_id : "");
            nvs_erase_key(my_handle, "result");
            nvs_commit(my_handle);
            nvs_close(my_handle);
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
