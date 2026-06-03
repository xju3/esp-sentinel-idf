#include "task_ota.h"
#include "config_manager.h"
#include "logger.h"
#include "machine_state.h"
#include "task_daq.h"
#include "task_fft.h"
#include "data_dispatcher.h"
#include "http_proxy.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// 引入 bsp_4g 中为 OTA 扩展的接口
extern esp_err_t bsp_4g_http_put(const char *url, const char *payload);
extern esp_err_t bsp_4g_ota_download_and_write(const char *url, int fw_size, const char *access_key, esp_ota_handle_t update_handle);

// 最长等待采样及计算队列排空的超时时间
#define OTA_DRAIN_TIMEOUT_MS 30000

// 内部方法：上报 OTA 结果 (HTTP PUT)
static void report_ota_result(const char *task_id, int result_code)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s", g_user_config.host, task_id);
    
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

void execute_ota_update_sync(const char *task_id)
{
    LOG_INFOF("=== Starting OTA Update Process for Task: %s ===", task_id);

    // 1. 停止触发新任务，准备等待流水线排空
    LOG_INFO("1. Pausing periodic DAQ tasks and waiting for pipeline drain...");
    task_daq_pause_periodic();

    TickType_t start_ticks = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(OTA_DRAIN_TIMEOUT_MS);
    bool drained = false;

    while ((xTaskGetTickCount() - start_ticks) < timeout_ticks) {
        if (task_daq_is_idle() && task_fft_is_idle()) {
            drained = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!drained) {
        LOG_WARN("Pipeline drain timeout! Proceeding to force OTA.");
    } else {
        LOG_INFO("Pipeline drained successfully.");
    }

    // 2. 将积压特征数据落袋为安，推送到云端
    LOG_INFO("2. Flushing remaining data to server...");
    data_dispatcher_flush_all(pdMS_TO_TICKS(5000));

    // 3. 加锁独占系统，屏蔽硬件扫描被以外唤醒
    LOG_INFO("3. Locking system task mutex for exclusive OTA operations...");
    lock_system_task();

    // 4. 向服务器获取固件元信息
    char url[256];
    char *json_response = NULL;
    snprintf(url, sizeof(url), "http://%s/sensors/ota/%s", g_user_config.host, task_id);
    
    LOG_INFOF("Fetching OTA info from: %s", url);
    if (http_proxy_get(url, &json_response) != ESP_OK || json_response == NULL) {
        LOG_ERROR("Failed to fetch OTA info.");
        report_ota_result(task_id, 1);
        unlock_system_task();
        task_daq_resume_periodic(true);
        return;
    }

    cJSON *root = cJSON_Parse(json_response);
    free(json_response);

    if (!root) {
        LOG_ERROR("Failed to parse OTA info JSON.");
        report_ota_result(task_id, 2);
        unlock_system_task();
        task_daq_resume_periodic(true);
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
        task_daq_resume_periodic(true);
        return;
    }

    int fw_size = size_item->valueint;
    const char *fw_url = path_item->valuestring;
    const char *access_key = cJSON_IsString(key_item) ? key_item->valuestring : "";
    LOG_INFOF("OTA Firmware Info: size=%d, url=%s", fw_size, fw_url);

    // 5. 执行下载与写分区
    esp_err_t ota_err = ESP_FAIL;
    
    if (g_user_config.network == 1) {
        // 4G 模式
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
        // WiFi 模式 (使用原生 esp_https_ota 会吃大内存，这里手写分块写入防 OOM)
        LOG_INFO("Starting OTA download via WiFi...");
        esp_http_client_config_t config = {
            .url = fw_url,
            .timeout_ms = 30000,
            .keep_alive_enable = true,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        
        if (client) {
            // 设置授权 Header，必须在 esp_http_client_open 之前设置
            if (access_key && access_key[0] != '\0') {
                esp_http_client_set_header(client, "Authorization", access_key);
            }
            
            if (esp_http_client_open(client, 0) == ESP_OK) {
                esp_http_client_fetch_headers(client);
                
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
                                } else if (data_read == 0) {
                                    LOG_INFO("Connection closed, all data received");
                                    break;
                                }
                            }
                            free(ota_write_data);
                        } else {
                            ota_success = false;
                            LOG_ERROR("OOM during OTA buffer allocation");
                        }
                        
                        if (ota_success && binary_file_length == fw_size) {
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

    cJSON_Delete(root);

    // 6. 汇报结果并根据情况重启系统
    if (ota_err == ESP_OK) {
        report_ota_result(task_id, 0); // 0 表示成功
        LOG_INFO("OTA Update completed successfully. System will restart in 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart(); // 重启应用新固件，如果失败底层回滚机制会自动切换回原分区
    } else {
        LOG_ERROR("OTA Update failed.");
        report_ota_result(task_id, 4); // 大于 0 表示异常
        unlock_system_task();          // 释放排他锁
        task_daq_resume_periodic(true);// 恢复原有的采样业务
    }
}