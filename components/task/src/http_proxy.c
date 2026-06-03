#include "http_proxy.h"
#include "config_manager.h"
#include "esp_http_client.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>

// 引入 4G BSP 接口
extern esp_err_t bsp_4g_http_get(const char *url, char **out_response);

esp_err_t http_proxy_get(const char *url, char **out_response)
{
    if (url == NULL || out_response == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *out_response = NULL;

    // 1 代表 4G 网络模式
    if (g_user_config.network == 1) {
        LOG_INFO("Using 4G AT mode for HTTP GET...");
        return bsp_4g_http_get(url, out_response);
    } else {
        // WiFi 网络模式，使用 ESP-IDF 原生 HTTP 客户端
        LOG_INFO("Using WiFi mode for HTTP GET...");
        esp_http_client_config_t config = {
            .url = url,
            .timeout_ms = 15000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            LOG_ERROR("Failed to initialize HTTP client");
            return ESP_FAIL;
        }

        esp_err_t err = esp_http_client_open(client, 0);
        if (err == ESP_OK) {
            esp_http_client_fetch_headers(client);
            int content_length = esp_http_client_get_content_length(client);
            
            if (content_length > 0) {
                *out_response = calloc(1, content_length + 1);
                if (*out_response) {
                    int read_len = esp_http_client_read(client, *out_response, content_length);
                    if (read_len != content_length) {
                        LOG_WARN("Incomplete HTTP read");
                        err = ESP_FAIL;
                        free(*out_response);
                        *out_response = NULL;
                    }
                } else {
                    LOG_ERROR("OOM allocating HTTP response buffer");
                    err = ESP_ERR_NO_MEM;
                }
            } else {
                // 如果是 Chunked 编码或未指定长度，动态增长读取
                int chunk_size = 512;
                int total_read = 0;
                char *buf = malloc(chunk_size);
                if (buf) {
                    while (1) {
                        int read_len = esp_http_client_read(client, buf + total_read, chunk_size - 1);
                        if (read_len <= 0) break;
                        total_read += read_len;
                        
                        char *new_buf = realloc(buf, total_read + chunk_size);
                        if (!new_buf) {
                            err = ESP_ERR_NO_MEM;
                            break;
                        }
                        buf = new_buf;
                    }
                    if (err == ESP_OK) {
                        buf[total_read] = '\0';
                        *out_response = buf;
                    } else {
                        free(buf);
                    }
                } else {
                    err = ESP_ERR_NO_MEM;
                }
            }
        } else {
            LOG_ERRORF("Failed to open HTTP connection: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);
        return err;
    }
}