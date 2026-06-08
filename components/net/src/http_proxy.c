#include "http_proxy.h"
#include "config_manager.h"
#include "esp_http_client.h"
#include "logger.h"
#include <string.h>
#include <stdlib.h>

// 底层 4G AT 指令实现 (实现在 bsp_4g.c 中)
extern esp_err_t bsp_4g_http_get(const char *url, char **out_response);
extern esp_err_t bsp_4g_http_post_json(const char *url, const char *payload, char **out_response);

esp_err_t http_proxy_get(const char *url, char **out_response)
{
    if (!url || !out_response) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_response = NULL;

    // --- 1. 如果是 4G 模式 ---
    if (g_user_config.network == 1) {
        return bsp_4g_http_get(url, out_response);
    }

    // --- 2. 如果是 WiFi 模式 ---
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        LOG_ERROR("Failed to initialize HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        int content_length = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        
        if (status_code == 200) {
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
                // Chunked 编码或未指定长度，动态增长读取
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
                            LOG_ERROR("OOM reallocating HTTP response buffer");
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
                    LOG_ERROR("OOM allocating HTTP response buffer");
                    err = ESP_ERR_NO_MEM;
                }
            }
        } else {
            LOG_ERRORF("HTTP GET failed with status code: %d", status_code);
            err = ESP_FAIL;
        }
    } else {
        LOG_ERRORF("Failed to open HTTP connection: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t http_client_read_response_body(esp_http_client_handle_t client, char **out_response)
{
    if (!out_response) {
        return ESP_OK;
    }
    *out_response = NULL;

    int content_length = esp_http_client_get_content_length(client);
    if (content_length > 0) {
        *out_response = calloc(1, content_length + 1);
        if (!*out_response) {
            LOG_ERROR("OOM allocating HTTP POST response buffer");
            return ESP_ERR_NO_MEM;
        }
        int read_len = esp_http_client_read(client, *out_response, content_length);
        if (read_len != content_length) {
            LOG_WARN("Incomplete HTTP POST response read");
            free(*out_response);
            *out_response = NULL;
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    const int chunk_size = 512;
    int total_read = 0;
    char *buf = malloc(chunk_size);
    if (!buf) {
        LOG_ERROR("OOM allocating HTTP POST response buffer");
        return ESP_ERR_NO_MEM;
    }

    while (1) {
        int read_len = esp_http_client_read(client, buf + total_read, chunk_size - 1);
        if (read_len <= 0) {
            break;
        }
        total_read += read_len;

        char *new_buf = realloc(buf, total_read + chunk_size);
        if (!new_buf) {
            LOG_ERROR("OOM reallocating HTTP POST response buffer");
            free(buf);
            return ESP_ERR_NO_MEM;
        }
        buf = new_buf;
    }

    buf[total_read] = '\0';
    *out_response = buf;
    return ESP_OK;
}

esp_err_t http_proxy_post_json(const char *url, const char *payload, char **out_response)
{
    if (!url || !payload) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_response) {
        *out_response = NULL;
    }

    if (g_user_config.network == 1) {
        return bsp_4g_http_post_json(url, payload, out_response);
    }

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        LOG_ERROR("Failed to initialize HTTP POST client");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    const int payload_len = strlen(payload);
    esp_err_t err = esp_http_client_open(client, payload_len);
    if (err == ESP_OK) {
        int written = esp_http_client_write(client, payload, payload_len);
        if (written != payload_len) {
            LOG_ERRORF("HTTP POST write failed: written=%d expected=%d", written, payload_len);
            err = ESP_FAIL;
        }
    }
    if (err == ESP_OK) {
        (void)esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        if (status_code < 200 || status_code >= 300) {
            LOG_ERRORF("HTTP POST failed with status code: %d", status_code);
            err = ESP_FAIL;
        } else {
            err = http_client_read_response_body(client, out_response);
            LOG_INFOF("HTTP POST completed, status=%d", status_code);
        }
    } else {
        LOG_ERRORF("HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}
