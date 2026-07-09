#include "task_ota.h"
#include "config_manager.h"
#include "logger.h"
#include "system_lock.h"
#include "task_daq.h"

#include "http_proxy.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "drv_4g.h"

extern esp_err_t bsp_4g_ota_download_and_write(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle);

// 最长等待采样及计算队列排空的超时时间
#define OTA_DRAIN_TIMEOUT_MS 30000

// 内部方法：上报 OTA 结果 (HTTP PUT)
static void report_ota_result(const char *task_id, int result_code)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s", g_user_config.api_host, task_id);
    
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"result\": %d}", result_code);
    
    LOG_INFOF("Reporting OTA result %d to %s", result_code, url);

    if (g_user_config.network == 1) {
        esp_err_t err = bsp_4g_http_put(url, payload);
        if (err == ESP_OK) {
            LOG_INFO("4G HTTP PUT result reported successfully.");
        } else {
            LOG_ERROR("4G HTTP PUT result report failed.");
        }
    } else {
        esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_PUT,
            .timeout_ms = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client) {
            esp_http_client_set_header(client, "Content-Type", "application/json");
            esp_http_client_set_post_field(client, payload, strlen(payload));
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                LOG_INFOF("Reported OTA result, HTTP status = %d", esp_http_client_get_status_code(client));
            } else {
                LOG_ERRORF("Failed to report OTA result: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(client);
        }
    }
}

// 内部方法：上报 OTA 最终完成结果 (HTTP POST /complete)
static void report_ota_complete(const char *task_id, int result_code)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/%s/complete/%d", g_user_config.api_host, task_id, result_code);
    
    LOG_INFOF("Reporting OTA complete result %d to %s", result_code, url);

    if (g_user_config.network == 1) {
        char *response = NULL;
        esp_err_t err = bsp_4g_http_post_json(url, "", &response);
        if (err == ESP_OK) {
            LOG_INFO("4G HTTP POST complete result reported successfully.");
            if (response) free(response);
        } else {
            LOG_ERROR("4G HTTP POST complete result report failed.");
        }
    } else {
        esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client) {
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                LOG_INFOF("Reported OTA complete, HTTP status = %d", esp_http_client_get_status_code(client));
            } else {
                LOG_ERRORF("Failed to report OTA complete: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(client);
        }
    }
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
            
            if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
                if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                    LOG_INFO("OTA verification pending. Marking app as valid.");
                    esp_ota_mark_app_valid_cancel_rollback();
                    report_ota_complete(task_id, 0); // 0: Success
                } else {
                    LOG_WARN("OTA rollback detected. Reporting failure.");
                    report_ota_complete(task_id, 1); // 1: Failure
                }
            } else {
                LOG_WARN("OTA state unavailable, assuming rollback or normal boot. Reporting failure.");
                report_ota_complete(task_id, 1); // 1: Failure
            }
            
            nvs_erase_key(my_handle, "task_id");
            nvs_commit(my_handle);
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

    if (g_user_config.network == 1) {
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
    } else {
        LOG_INFO("Starting OTA download via WiFi...");
        esp_http_client_config_t config = {
            .url = fw_url,
            .timeout_ms = 30000,
            .keep_alive_enable = true,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        if (client) {
            if (access_key && access_key[0] != '\0') {
                esp_http_client_set_header(client, "Authorization", access_key);
            }

            if (esp_http_client_open(client, 0) == ESP_OK) {
                (void)esp_http_client_fetch_headers(client);

                const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
                if (update_partition) {
                    LOG_INFOF("Writing to partition subtype %d at offset 0x%lx", update_partition->subtype, update_partition->address);
                    esp_ota_handle_t update_handle = 0;

                    if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) == ESP_OK) {
                        char *ota_write_data = malloc(4096);
                        int binary_file_length = 0;
                        bool ota_success = true;

                        if (ota_write_data) {
                            while (1) {
                                int data_read = esp_http_client_read(client, ota_write_data, 4096);
                                if (data_read < 0) {
                                    LOG_ERROR("Error: data read error");
                                    ota_success = false;
                                    break;
                                } else if (data_read > 0) {
                                    if (esp_ota_write(update_handle, (const void *)ota_write_data, data_read) != ESP_OK) {
                                        LOG_ERROR("Error: esp_ota_write failed");
                                        ota_success = false;
                                        break;
                                    }
                                    binary_file_length += data_read;
                                } else {
                                    LOG_INFO("Connection closed, all data received");
                                    break;
                                }
                            }
                            free(ota_write_data);
                        } else {
                            ota_success = false;
                            LOG_ERROR("OOM during OTA buffer allocation");
                        }

                        const bool size_ok = (fw_size <= 0 && binary_file_length > 0) ||
                                             (fw_size > 0 && binary_file_length == fw_size);
                        if (ota_success && size_ok) {
                            if (esp_ota_end(update_handle) == ESP_OK) {
                                if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                                    ota_err = ESP_OK;
                                    LOG_INFO("OTA Success! Boot partition configured.");
                                }
                            }
                        } else {
                            LOG_ERROR("OTA Aborted due to read error or size mismatch.");
                            esp_ota_abort(update_handle);
                        }
                    }
                }
            }
            esp_http_client_cleanup(client);
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
