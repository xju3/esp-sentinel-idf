#ifndef REPORT_PIPELINE_H
#define REPORT_PIPELINE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct report_payload report_payload_t;

/** Capture sensor data, calculate features, and retain the report JSON. */
esp_err_t report_pipeline_capture(const char *task_id,
                                  report_payload_t **out_payload);

/** Upload and release one payload returned by report_pipeline_capture(). */
esp_err_t report_pipeline_upload(report_payload_t *payload);

/** Release a captured payload without uploading it. */
void report_pipeline_discard(report_payload_t *payload);

#ifdef __cplusplus
}
#endif

#endif // REPORT_PIPELINE_H
