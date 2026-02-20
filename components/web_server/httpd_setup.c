#include "web_server.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"

static const char *TAG = "web_server";

void captive_dns_start(void);
void captive_dns_stop(void);

static httpd_handle_t s_server = NULL;

static bool file_exists(const char *path)
{
    struct stat st;
    return (path && stat(path, &st) == 0 && S_ISREG(st.st_mode));
}

static esp_err_t ensure_spiffs_mounted(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/system",
        .partition_label = "system",
        .max_files = 8,
        .format_if_mount_failed = false,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK; // already mounted
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

static const char *guess_content_type(const char *path)
{
    const char *ext = strrchr(path, '.');
    if (!ext) return "text/plain";
    if (strcmp(ext, ".html") == 0) return "text/html";
    if (strcmp(ext, ".css") == 0) return "text/css";
    if (strcmp(ext, ".js") == 0) return "application/javascript";
    if (strcmp(ext, ".svg") == 0) return "image/svg+xml";
    if (strcmp(ext, ".json") == 0) return "application/json";
    if (strcmp(ext, ".ico") == 0) return "image/x-icon";
    return "text/plain";
}

static esp_err_t send_file(httpd_req_t *req, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, guess_content_type(path));

    char buf[1024];
    size_t read_bytes = 0;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK) {
            fclose(f);
            httpd_resp_sendstr_chunk(req, NULL);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void redirect_to_root(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, "", 0);
}

static esp_err_t captive_get_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    // Captive portal detection endpoints
    if (strcmp(uri, "/generate_204") == 0 ||
        strcmp(uri, "/connecttest.txt") == 0 ||
        strcmp(uri, "/ncsi.txt") == 0 ||
        strcmp(uri, "/hotspot-detect.html") == 0 ||
        strcmp(uri, "/success.txt") == 0) {
        redirect_to_root(req);
        return ESP_OK;
    }

    // Root or any path -> try static file, fallback to index.html
    const char *base = "/system/www";
    char path[256];

    if (strcmp(uri, "/") == 0) {
        snprintf(path, sizeof(path), "%s/index.html", base);
        return send_file(req, path);
    }

    // Strip query string if present
    char uri_copy[128];
    uri_copy[0] = '\0';
    if (uri) {
        // Avoid -Wformat-truncation by limiting copy length
        snprintf(uri_copy, sizeof(uri_copy), "%.*s", (int)sizeof(uri_copy) - 1, uri);
    }
    char *q = strchr(uri_copy, '?');
    if (q) *q = '\0';

    snprintf(path, sizeof(path), "%s%s", base, uri_copy);
    if (file_exists(path)) {
        return send_file(req, path);
    }

    // Fallback to index.html for SPA routing
    snprintf(path, sizeof(path), "%s/index.html", base);
    return send_file(req, path);
}

esp_err_t web_server_start(void)
{
    if (s_server) {
        return ESP_OK;
    }

    esp_err_t err = ensure_spiffs_mounted();
    if (err != ESP_OK) {
        return err;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    httpd_uri_t catch_all = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_get_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_server, &catch_all);

    captive_dns_start();

    ESP_LOGI(TAG, "Web server started");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (!s_server) {
        return;
    }
    httpd_stop(s_server);
    s_server = NULL;
    captive_dns_stop();
}
