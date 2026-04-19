#ifndef OFF_SLEEP_MANAGER_H
#define OFF_SLEEP_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t start_off_sleep_manager(void);
esp_err_t off_sleep_manager_request_sleep(void);
bool off_sleep_manager_sleep_requested(void);

#ifdef __cplusplus
}
#endif

#endif // OFF_SLEEP_MANAGER_H
