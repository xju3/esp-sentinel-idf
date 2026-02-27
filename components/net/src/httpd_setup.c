#include "web_server.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "api_handlers.h"
#include "logger.h"

static httpd_handle_t s_server = NULL;
static bool s_server_was_running_before_scan = false;

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
    if (err == ESP_ERR_INVALID_STATE)
    {
        return ESP_OK; // already mounted
    }
    if (err != ESP_OK)
    {
        LOG_ERRORF("Failed to mount SPIFFS: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

static const char *guess_content_type(const char *path)
{
    const char *ext = strrchr(path, '.');
    if (!ext)
        return "text/plain";
    if (strcmp(ext, ".html") == 0)
        return "text/html";
    if (strcmp(ext, ".css") == 0)
        return "text/css";
    if (strcmp(ext, ".js") == 0)
        return "application/javascript";
    if (strcmp(ext, ".svg") == 0)
        return "image/svg+xml";
    if (strcmp(ext, ".json") == 0)
        return "application/json";
    if (strcmp(ext, ".ico") == 0)
        return "image/x-icon";
    return "text/plain";
}

static esp_err_t send_file(httpd_req_t *req, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, guess_content_type(path));

    char buf[1024];
    size_t read_bytes = 0;
    while ((read_bytes = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK)
        {
            fclose(f);
            // Abort immediately if sending fails (e.g. client disconnected)
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
        strcmp(uri, "/success.txt") == 0)
    {
        redirect_to_root(req);
        return ESP_OK;
    }

    // Root or any path -> try static file, fallback to index.html
    const char *base = "/system/w";
    char path[256];

    if (strcmp(uri, "/") == 0)
    {
        snprintf(path, sizeof(path), "%s/index.html", base);
        return send_file(req, path);
    }

    // Strip query string if present
    char uri_copy[128];
    uri_copy[0] = '\0';
    if (uri)
    {
        // Avoid -Wformat-truncation by limiting copy length
        snprintf(uri_copy, sizeof(uri_copy), "%.*s", (int)sizeof(uri_copy) - 1, uri);
    }
    char *q = strchr(uri_copy, '?');
    if (q)
        *q = '\0';

    snprintf(path, sizeof(path), "%s%s", base, uri_copy);
    if (file_exists(path))
    {
        return send_file(req, path);
    }

    // Fallback to index.html for SPA routing
    snprintf(path, sizeof(path), "%s/index.html", base);
    return send_file(req, path);
}

esp_err_t web_server_start(void)
{
    if (s_server)
    {
        return ESP_OK;
    }

    esp_err_t err = ensure_spiffs_mounted();
    if (err != ESP_OK)
    {
        return err;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) != ESP_OK)
    {
        LOG_ERROR("Failed to start HTTP server");
        return ESP_FAIL;
    }

    // API endpoints
    httpd_uri_t api_load_config = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = api_get_config_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t api_save_config = {
        .uri = "/api/save",
        .method = HTTP_POST,
        .handler = api_save_config_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t api_wifi_list = {
        .uri = "/api/wifi-list",
        .method = HTTP_GET,
        .handler = api_wifi_list_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t api_wifi_scan = {
        .uri = "/api/wifi-scan-start",
        .method = HTTP_GET,
        .handler = api_wifi_scan_start_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t api_consumption = {
        .uri = "/api/consumption",
        .method = HTTP_GET,
        .handler = api_get_consumption_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t catch_all = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t catch_all_post = {
        .uri = "/*",
        .method = HTTP_POST,
        .handler = captive_get_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(s_server, &api_load_config);
    httpd_register_uri_handler(s_server, &api_save_config);
    httpd_register_uri_handler(s_server, &api_wifi_list);
    httpd_register_uri_handler(s_server, &api_wifi_scan);
    httpd_register_uri_handler(s_server, &api_consumption);
    httpd_register_uri_handler(s_server, &catch_all);
    httpd_register_uri_handler(s_server, &catch_all_post);

    LOG_INFO("Web server started");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (!s_server)
    {
        return;
    }
    httpd_stop(s_server);
    s_server = NULL;
}

bool web_server_is_running(void)
{
    return s_server != NULL;
}

void web_server_mark_scan_start(void)
{
    s_server_was_running_before_scan = web_server_is_running();
}

void web_server_mark_scan_done(void)
{
    if (s_server_was_running_before_scan)
    {
        web_server_start();
    }
    s_server_was_running_before_scan = false;
}
