#ifndef TASK_OTA_H
#define TASK_OTA_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void execute_ota_update_sync(const char *task_id);
void execute_ota_update_from_url_sync(const char *task_id, const char *fw_url);
/** Confirm a newly booted OTA image locally and persist its report result. */
esp_err_t task_ota_finalize_boot_status(void);
/** Report a persisted OTA result after the current sensor report succeeds. */
void task_ota_report_pending_completion(void);
/** True while this OTA task has completed locally but is not reported yet. */
bool task_ota_completion_pending_for(const char *task_id);

#ifdef __cplusplus
}
#endif
#endif // TASK_OTA_H
