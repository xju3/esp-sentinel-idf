#ifndef OFF_SLEEP_MANAGER_H
#define OFF_SLEEP_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

#if LIS2

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t start_off_sleep_manager(void);
esp_err_t off_sleep_manager_request_sleep(void);
bool off_sleep_manager_sleep_requested(void);

#ifdef __cplusplus
}
#endif

#else // LIS2 == 0: provide no-op stubs

static inline esp_err_t start_off_sleep_manager(void)          { return ESP_OK; }
static inline esp_err_t off_sleep_manager_request_sleep(void)  { return ESP_ERR_NOT_SUPPORTED; }
static inline bool      off_sleep_manager_sleep_requested(void){ return false; }

#endif // LIS2

#endif // OFF_SLEEP_MANAGER_H
