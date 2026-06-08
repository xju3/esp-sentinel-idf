#ifndef REPORT_PIPELINE_H
#define REPORT_PIPELINE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t report_pipeline_run(const char *task_id);

#ifdef __cplusplus
}
#endif

#endif // REPORT_PIPELINE_H
