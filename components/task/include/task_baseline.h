#ifndef APP_BASELINE_H
#define APP_BASELINE_H

#include <stdbool.h>
#include <stddef.h>

#include "cJSON.h"
#include "esp_err.h"
#include "task_stash.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t set_device_baseline(const char *device_id);

#ifdef __cplusplus
}
#endif

#endif // APP_BASELINE_H
