#ifndef TASK_OTA_H
#define TASK_OTA_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void execute_ota_update_sync(const char *task_id);
void execute_ota_update_from_url_sync(const char *task_id, const char *fw_url);
bool task_ota_status_pending(void);
void check_and_report_ota_status(void);

#ifdef __cplusplus
}
#endif
#endif // TASK_OTA_H
