#include "server_report_task_scheduler.h"

#include "esp_attr.h"
#include "logger.h"

#include <string.h>

#define SERVER_REPORT_TASK_ID_MAX 64
#define SERVER_REPORT_TASK_MIN_ACTION 10
#define SERVER_REPORT_TASK_MAX_ACTION 99

RTC_DATA_ATTR static bool s_task_active = false;
RTC_DATA_ATTR static char s_task_id[SERVER_REPORT_TASK_ID_MAX] = {0};
RTC_DATA_ATTR static int64_t s_task_interval_us = 0;
RTC_DATA_ATTR static int64_t s_task_left_us = 0;
RTC_DATA_ATTR static int s_task_remaining = 0;

static bool s_task_scheduled_this_run = false;

static bool is_valid_task_request(const char *task_id, int action, int val, int *out_interval_min)
{
    if (!task_id || task_id[0] == '\0') {
        return false;
    }
    if (action < SERVER_REPORT_TASK_MIN_ACTION || action > SERVER_REPORT_TASK_MAX_ACTION) {
        return false;
    }
    const int interval_min = action % 10;
    if (interval_min <= 0 || val <= 0) {
        return false;
    }
    if (out_interval_min) {
        *out_interval_min = interval_min;
    }
    return true;
}

void server_report_task_on_wake(int64_t slept_us)
{
    s_task_scheduled_this_run = false;
    if (!s_task_active || slept_us <= 0) {
        return;
    }
    s_task_left_us -= slept_us;
}

void server_report_task_after_work(int64_t work_us)
{
    if (!s_task_active || work_us <= 0) {
        return;
    }
    if (s_task_scheduled_this_run) {
        s_task_scheduled_this_run = false;
        return;
    }
    s_task_left_us -= work_us;
}

void server_report_task_clear(const char *reason)
{
    if (s_task_active) {
        LOG_WARNF("Clearing server report task: id=%s, remaining=%d, reason=%s",
                  s_task_id,
                  s_task_remaining,
                  reason ? reason : "");
    }
    s_task_active = false;
    s_task_id[0] = '\0';
    s_task_interval_us = 0;
    s_task_left_us = 0;
    s_task_remaining = 0;
    s_task_scheduled_this_run = false;
}

bool server_report_task_is_active(void)
{
    return s_task_active;
}

bool server_report_task_is_due(void)
{
    return s_task_active && s_task_remaining > 0 && s_task_left_us <= 0;
}

bool server_report_task_copy_due_id(char *out_task_id, size_t out_len)
{
    if (!out_task_id || out_len == 0 || !server_report_task_is_due()) {
        return false;
    }
    strlcpy(out_task_id, s_task_id, out_len);
    return true;
}

void server_report_task_mark_attempted(const char *task_id)
{
    if (!s_task_active || !task_id || strcmp(task_id, s_task_id) != 0) {
        return;
    }

    if (s_task_remaining > 0) {
        --s_task_remaining;
    }

    if (s_task_remaining <= 0) {
        LOG_INFOF("Server report task completed: id=%s", s_task_id);
        server_report_task_clear("completed");
        return;
    }

    s_task_left_us += s_task_interval_us;
    LOG_INFOF("Server report task attempt done: id=%s, remaining=%d, next_in_ms=%lld",
              s_task_id,
              s_task_remaining,
              (long long)(s_task_left_us / 1000LL));
}

int64_t server_report_task_left_us(void)
{
    return s_task_active ? s_task_left_us : INT64_MAX;
}

bool server_report_task_schedule(const char *task_id, int action, int val)
{
    int interval_min = 0;
    if (!is_valid_task_request(task_id, action, val, &interval_min)) {
        LOG_WARNF("Ignoring invalid server report task: id=%s, action=%d, val=%d",
                  task_id ? task_id : "",
                  action,
                  val);
        return false;
    }

    strlcpy(s_task_id, task_id, sizeof(s_task_id));
    s_task_interval_us = (int64_t)interval_min * 60000000LL;
    s_task_left_us = s_task_interval_us;
    s_task_remaining = val;
    s_task_active = true;
    s_task_scheduled_this_run = true;

    LOG_INFOF("Scheduled server report task: id=%s, action=%d, interval_min=%d, count=%d",
              s_task_id,
              action,
              interval_min,
              s_task_remaining);
    return true;
}

bool server_report_task_schedule_from_response(const cJSON *data)
{
    if (!cJSON_IsArray(data)) {
        return false;
    }

    cJSON *item = NULL;
    cJSON_ArrayForEach(item, data)
    {
        const cJSON *task_id = cJSON_GetObjectItemCaseSensitive(item, "id");
        const cJSON *action = cJSON_GetObjectItemCaseSensitive(item, "action");
        const cJSON *val = cJSON_GetObjectItemCaseSensitive(item, "val");
        if (!cJSON_IsString(task_id) || !cJSON_IsNumber(action) || !cJSON_IsNumber(val)) {
            continue;
        }
        if (server_report_task_schedule(task_id->valuestring, action->valueint, val->valueint)) {
            return true;
        }
    }

    return false;
}
