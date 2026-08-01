#ifndef SERVER_REPORT_TASK_SCHEDULER_H
#define SERVER_REPORT_TASK_SCHEDULER_H

#include "cJSON.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void server_report_task_on_wake(int64_t slept_us);
void server_report_task_after_work(int64_t work_us);
void server_report_task_clear(const char *reason);
bool server_report_task_is_active(void);
bool server_report_task_is_due(void);
bool server_report_task_copy_due_id(char *out_task_id, size_t out_len);
int server_report_task_due_action(void);
void server_report_task_mark_attempted(const char *task_id);
int64_t server_report_task_left_us(void);
bool server_report_task_schedule(const char *task_id, int action, int val);
bool server_report_task_schedule_from_response(const cJSON *data);

#ifdef __cplusplus
}
#endif

#endif // SERVER_REPORT_TASK_SCHEDULER_H
