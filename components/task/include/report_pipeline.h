#ifndef REPORT_PIPELINE_H
#define REPORT_PIPELINE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t points;
    uint16_t range_g;
    bool upload_spectrum;
} report_pipeline_options_t;

esp_err_t report_pipeline_run(const char *task_id);
esp_err_t report_pipeline_run_with_options(const char *task_id, const report_pipeline_options_t *options);

#ifdef __cplusplus
}
#endif

#endif // REPORT_PIPELINE_H
