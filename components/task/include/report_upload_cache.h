#ifndef REPORT_UPLOAD_CACHE_H
#define REPORT_UPLOAD_CACHE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*report_upload_cache_sender_t)(const char *json, void *ctx);

esp_err_t report_upload_cache_save_failed(const char *json);
esp_err_t report_upload_cache_flush(report_upload_cache_sender_t sender, void *ctx);

#ifdef __cplusplus
}
#endif

#endif // REPORT_UPLOAD_CACHE_H
