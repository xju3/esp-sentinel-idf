#ifndef API_HANDLERS_H
#define API_HANDLERS_H

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t api_get_config_handler(httpd_req_t *req);
esp_err_t api_save_config_handler(httpd_req_t *req);
esp_err_t api_wifi_list_handler(httpd_req_t *req);
esp_err_t api_wifi_scan_start_handler(httpd_req_t *req);
esp_err_t api_get_consumption_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif // API_HANDLERS_H
