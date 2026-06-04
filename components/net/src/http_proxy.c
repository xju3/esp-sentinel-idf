#include "http_proxy.h"
#include "config_manager.h"
#include "esp_http_client.h"
#include "logger.h"
#include <string.h>
#include <stdlib.h>

// 底层 4G AT 指令实现 (实现在 bsp_4g.c 中)
extern esp_err_t bsp_4g_http_get(const char *url, char **out_response);

esp_err_t http_proxy_get(const char *url, char **out_response)
{
    if (!url || !out_response) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_response = NULL;

    // --- 1. 如果是 4G 模式 ---
    if (g_user_config.network == 1) {
        // LOG_INFO("HTTP GET routing via 4G AT Mode...");
        return bsp_4g_http_get(url, out_response);
    }

    // --- 2. 如果是 WiFi 模式 ---
    // LOG_INFO("HTTP GET routing via WiFi LwIP...");
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        int content_length = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            int buffer_size = (content_length > 0) ? content_length : 1024;
            char *buffer = calloc(1, buffer_size + 1);
            int read_bytes = 0;
            while (buffer != NULL) {
                int r = esp_http_client_read(client, buffer + read_bytes, buffer_size - read_bytes);
                if (r < 0) { free(buffer); buffer = NULL; break; }
                if (r == 0) break;
                read_bytes += r;
                if (read_bytes == buffer_size) {
                    buffer_size *= 2; // 如果 content_length 未知，动态扩容
                    buffer = realloc(buffer, buffer_size + 1);
                }
            }
            if (buffer) {
                buffer[read_bytes] = '\0';
                *out_response = buffer;
            } else {
                err = ESP_FAIL;
            }
        } else {
            err = ESP_FAIL;
        }
    }
    esp_http_client_cleanup(client);
    return err;
}