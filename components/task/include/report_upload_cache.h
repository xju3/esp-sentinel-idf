#ifndef REPORT_UPLOAD_CACHE_H
#define REPORT_UPLOAD_CACHE_H

#include "esp_err.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*report_upload_cache_sender_t)(const char *json, void *ctx);

/** Persist one report as /user/<rtc_time_s>.json without fixing its delay. */
esp_err_t report_upload_cache_store(const char *json, uint64_t *out_time_s);

/** Get the total number of cached reports pending upload. */
uint32_t report_upload_cache_get_count(void);

/** Upload oldest files first, assigning delay by distance from the current report. */
esp_err_t report_upload_cache_flush(report_upload_cache_sender_t sender,
                                    void *ctx);

#ifdef __cplusplus
}
#endif

#endif // REPORT_UPLOAD_CACHE_H
