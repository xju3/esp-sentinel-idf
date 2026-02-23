#ifndef ICM4288P_BASELINE_H
#define ICM4288P_BASELINE_H

#include <stdbool.h>
#include "esp_err.h"
#include "icm4288p.h"
#include "config_manager.h"

typedef struct {
    float val;
    float offset;
} icm_axis_freq_t;

typedef struct {
    icm_axis_freq_t x;
    icm_axis_freq_t y;
    icm_axis_freq_t z;
} icm_freq_profile_t;

// Global baseline profile available to sampling/analysis tasks after init.
extern icm_freq_profile_t g_icm_baseline;

// Load or create baseline for the given device id. Blocking sampling at 1 kHz if missing.
esp_err_t icm4288p_ensure_baseline(const char *device_id, icm_freq_profile_t *out_profile);

#endif // ICM4288P_BASELINE_H
