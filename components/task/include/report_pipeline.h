#ifndef REPORT_PIPELINE_H
#define REPORT_PIPELINE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct report_payload report_payload_t;
typedef void (*report_sample_complete_fn)(void *ctx);

/** Capture sensor data, calculate features, and retain the report JSON. */
esp_err_t report_pipeline_capture(const char *task_id,
                                  report_payload_t **out_payload);

/**
 * Capture a report and invoke a hook after the final raw sample has been
 * accepted, before FFT/feature calculation and JSON construction begin.
 */
esp_err_t report_pipeline_capture_with_sample_complete(
    const char *task_id,
    report_sample_complete_fn sample_complete,
    void *sample_complete_ctx,
    report_payload_t **out_payload);

/** Upload and release one payload returned by report_pipeline_capture(). */
esp_err_t report_pipeline_upload(report_payload_t *payload);

/** Persist one payload for ordered retry without releasing it. */
esp_err_t report_pipeline_cache(report_payload_t *payload);

/** Retry persisted reports after all current reports uploaded successfully. */
esp_err_t report_pipeline_flush_cache(void);

/** Release a captured payload without uploading it. */
void report_pipeline_discard(report_payload_t *payload);

#ifdef __cplusplus
}
#endif

#endif // REPORT_PIPELINE_H
