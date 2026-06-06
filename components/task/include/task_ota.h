#ifndef TASK_OTA_H
#define TASK_OTA_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void execute_ota_update_sync(const char *task_id);

#ifdef __cplusplus
}
#endif
#endif // TASK_OTA_H