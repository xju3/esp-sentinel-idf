#ifndef APP_BASELINE_H
#define APP_BASELINE_H

#include <stdbool.h>
#include <stddef.h>

#include "cJSON.h"
#include "esp_err.h"
#include "vib_baseline.h"
#include "vib_welford_feature.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t app_baseline_capture(StreamBufferHandle_t stream, 
                               uint32_t duration_ms, 
                               vib_baseline_t *out_bl, const char *device_id); 

#ifdef __cplusplus
}
#endif

#endif // APP_BASELINE_H
