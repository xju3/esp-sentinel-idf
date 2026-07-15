/**
 * @file daq_scheduler.c (由原 task_daq.c 重构而来)
 * @brief 基于深度睡眠和RTC的DAQ一次性调度器
 */
#include "task_daq.h"
#include "config_manager.h"
#include "esp_err.h"
#include "esp_rtc_time.h"
#include "esp_sleep.h"
#include "logger.h"
#include "report_pipeline.h"
#include "server_report_task_scheduler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include <stddef.h>

// 容许提前唤醒的宽容度 (防止硬件定时器一点点抖动导致错过判断)
#define WAKEUP_TOLERANCE_US 2000000LL
#define IMMEDIATE_WAKEUP_US 1000000LL
#define NETWORK_PREP_TASK_STACK_SIZE 4096
#define NETWORK_PREP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

// 使用 RTC 内存维护相对倒计时，彻底隔绝 NTP 服务器挂钟时间的跳变
RTC_DATA_ATTR static int64_t s_patrol_left_us = 0;
RTC_DATA_ATTR static uint64_t s_last_sleep_rtc_us = 0;

// 本次唤醒周期的RTC起点；每次启动都会由 daq_scheduler_prepare() 设置。
static uint64_t s_cycle_start_rtc_us = 0;

// 本次运行中，哪些任务被触发执行了
static bool s_ran_patrol = false;
static bool s_is_wom_wakeup = false;

typedef struct {
  daq_before_upload_fn prepare;
  void *prepare_ctx;
  SemaphoreHandle_t done;
  esp_err_t result;
  int64_t started_us;
  int64_t finished_us;
  bool started;
  bool joined;
} async_network_prepare_t;

static void network_prepare_task(void *ctx) {
  async_network_prepare_t *state = (async_network_prepare_t *)ctx;
  state->result = state->prepare(state->prepare_ctx);
  state->finished_us = esp_timer_get_time();
  xSemaphoreGive(state->done);
  vTaskDelete(NULL);
}

static void start_network_after_sampling(void *ctx) {
  async_network_prepare_t *state = (async_network_prepare_t *)ctx;
  if (!state || state->started || state->joined || !state->done) {
    return;
  }

  state->started_us = esp_timer_get_time();
  state->started = true;
  BaseType_t created =
      xTaskCreate(network_prepare_task, "network_prepare",
                  NETWORK_PREP_TASK_STACK_SIZE, state,
                  NETWORK_PREP_TASK_PRIORITY, NULL);
  if (created != pdPASS) {
    state->started = false;
    state->started_us = 0;
    LOG_WARN("Could not start parallel 4G preparation; using synchronous fallback");
    return;
  }

  LOG_INFO("IIS3DWB sampling complete; starting 4G preparation in parallel with report processing");
}

static esp_err_t join_network_prepare(async_network_prepare_t *state,
                                      bool start_if_needed) {
  if (!state || state->joined) {
    return state ? state->result : ESP_ERR_INVALID_ARG;
  }

  if (!state->started) {
    if (!start_if_needed) {
      return ESP_OK;
    }
    state->result = state->prepare(state->prepare_ctx);
    state->joined = true;
    return state->result;
  }

  const int64_t wait_started_us = esp_timer_get_time();
  if (xSemaphoreTake(state->done, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }
  state->joined = true;

  const int64_t overlap_end_us = state->finished_us < wait_started_us
                                     ? state->finished_us
                                     : wait_started_us;
  const uint32_t overlap_ms = overlap_end_us > state->started_us
                                  ? (uint32_t)((overlap_end_us - state->started_us) / 1000LL)
                                  : 0U;
  const uint32_t wait_ms = state->finished_us > wait_started_us
                               ? (uint32_t)((state->finished_us - wait_started_us) / 1000LL)
                               : 0U;
  LOG_INFOF("Parallel 4G preparation: report_processing_overlap=%lu ms, final_wait=%lu ms",
            (unsigned long)overlap_ms, (unsigned long)wait_ms);
  return state->result;
}

/**
 * @brief 从深度睡眠唤醒后调用的 DAQ 评估入口
 * 此函数不依赖网络或绝对时间，只使用设备RTC相对计时
 */
esp_err_t daq_scheduler_prepare(bool *out_has_report_work) {
  if (!out_has_report_work) {
    return ESP_ERR_INVALID_ARG;
  }

  s_ran_patrol = false;

  const uint64_t rtc_now_us = esp_rtc_get_time_us();
  s_cycle_start_rtc_us = rtc_now_us;
  const bool woke_from_deep_sleep =
      (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED);

  if (!woke_from_deep_sleep || s_last_sleep_rtc_us == 0 ||
      rtc_now_us < s_last_sleep_rtc_us) {
    LOG_INFO("Cold boot detected. Initializing relative DAQ schedule.");
    s_patrol_left_us = 0;
    s_last_sleep_rtc_us = rtc_now_us;
  } else {
    // 使用RTC实际经过时间，而不是上次计划的睡眠时间。
    const int64_t slept_us = (int64_t)(rtc_now_us - s_last_sleep_rtc_us);
    s_patrol_left_us -= slept_us;
    server_report_task_on_wake(slept_us);
  }

  // 检查配置，>0 代表任务启用
  bool patrol_enabled = (g_user_config.patrol > 0);

  // 若禁用，将倒计时置为极大值防止误触发
  if (!patrol_enabled)
    s_patrol_left_us = INT64_MAX;

  if (patrol_enabled && s_patrol_left_us <= WAKEUP_TOLERANCE_US) {
    s_ran_patrol = true;
  }

  *out_has_report_work =
      s_is_wom_wakeup || s_ran_patrol || server_report_task_is_due();
  if (!*out_has_report_work) {
    LOG_INFO("Woke up but no report task scheduled to run right now.");
  }
  return ESP_OK;
}

esp_err_t daq_scheduler_execute(daq_before_upload_fn prepare_upload,
                                void *ctx) {
  if (!prepare_upload) {
    return ESP_ERR_INVALID_ARG;
  }

  typedef struct {
    report_payload_t *payload;
    bool server_task;
    char task_id[64];
  } pending_report_t;

  pending_report_t reports[2] = {0};
  size_t report_count = 0;
  esp_err_t err = ESP_OK;
  async_network_prepare_t network_prepare = {
      .prepare = prepare_upload,
      .prepare_ctx = ctx,
      .done = xSemaphoreCreateBinary(),
      .result = ESP_ERR_INVALID_STATE,
  };

  const bool run_wom_report = s_is_wom_wakeup;
  const bool run_patrol_report = s_ran_patrol;
  const bool run_server_report =
      !run_patrol_report && server_report_task_is_due();

  if (run_wom_report) {
    LOG_INFO("Executing WoM wakeup report pipeline...");
    const bool is_last_capture = !run_patrol_report && !run_server_report;
    err = report_pipeline_capture_with_sample_complete(
        "0", is_last_capture ? start_network_after_sampling : NULL,
        &network_prepare, &reports[report_count].payload);
    s_is_wom_wakeup = false;
    if (err != ESP_OK)
      goto cleanup;
    ++report_count;
  }

  if (run_patrol_report) {
    server_report_task_clear("normal report due");
    // LOG_INFO("Executing unified normal report pipeline...");
    err = report_pipeline_capture_with_sample_complete(
        NULL, start_network_after_sampling, &network_prepare,
        &reports[report_count].payload);
    if (err != ESP_OK)
      goto cleanup;
    ++report_count;
  } else if (run_server_report) {
    pending_report_t *report = &reports[report_count];
    if (server_report_task_copy_due_id(report->task_id,
                                       sizeof(report->task_id))) {
      LOG_INFOF("Executing scheduled server report task: id=%s",
                report->task_id);
      err = report_pipeline_capture_with_sample_complete(
          report->task_id, start_network_after_sampling, &network_prepare,
          &report->payload);
      if (err != ESP_OK)
        goto cleanup;
      report->server_task = true;
      ++report_count;
    }
  }

  if (report_count == 0) {
    LOG_INFO("Woke up but no report task scheduled to run right now.");
    if (network_prepare.done) {
      vSemaphoreDelete(network_prepare.done);
    }
    return ESP_OK;
  }

  // The modem starts only after the final raw capture. Feature calculation and
  // JSON construction run in parallel, then both paths join before upload.
  err = join_network_prepare(&network_prepare, true);
  if (err != ESP_OK)
    goto cleanup;

  for (size_t i = 0; i < report_count; ++i) {
    err = report_pipeline_upload(reports[i].payload);
    reports[i].payload = NULL;
    if (reports[i].server_task) {
      server_report_task_mark_attempted(reports[i].task_id);
    }
    if (err != ESP_OK)
      goto cleanup;
  }

cleanup:
  if (network_prepare.started && !network_prepare.joined) {
    (void)join_network_prepare(&network_prepare, false);
  }
  if (network_prepare.done) {
    vSemaphoreDelete(network_prepare.done);
  }

  for (size_t i = 0; i < report_count; ++i) {
    if (err != ESP_OK && reports[i].payload) {
      esp_err_t cache_err = report_pipeline_cache(reports[i].payload);
      if (cache_err == ESP_OK && reports[i].server_task) {
        server_report_task_mark_attempted(reports[i].task_id);
      } else if (cache_err != ESP_OK) {
        LOG_ERRORF("Failed to persist pending report: %s",
                   esp_err_to_name(cache_err));
      }
    }
    report_pipeline_discard(reports[i].payload);
  }
  return err;
}

/**
 * @brief 获取下一次需要唤醒的时间差（微秒），用于喂给
 * esp_sleep_enable_timer_wakeup
 */
uint64_t daq_scheduler_get_sleep_time_us(void) {
  // 1. 只使用设备RTC计算本轮实际工作耗时，不依赖基站/NTP时间。
  const uint64_t rtc_now_us = esp_rtc_get_time_us();
  int64_t exec_time_us = 0;
  if (s_cycle_start_rtc_us > 0 && rtc_now_us >= s_cycle_start_rtc_us) {
    exec_time_us = (int64_t)(rtc_now_us - s_cycle_start_rtc_us);
  }

  // 保存即将进入深睡前的RTC读数；下次唤醒后用差值获得实际睡眠时长。
  s_last_sleep_rtc_us = rtc_now_us;

  bool patrol_enabled = (g_user_config.patrol > 0);

  int64_t p_period_us =
      (int64_t)(g_user_config.patrol * 60000000.0); // 分钟 -> 微秒

  // 2. 将之前剩余的倒计时，继续扣减掉本次的工作耗时
  s_patrol_left_us -= exec_time_us;
  server_report_task_after_work(exec_time_us);

  const bool normal_overdue_after_work =
      (!s_ran_patrol && patrol_enabled && s_patrol_left_us <= 0);
  if (normal_overdue_after_work) {
    server_report_task_clear("normal report overdue after server task");
    return (uint64_t)IMMEDIATE_WAKEUP_US;
  }

  // 3. 对刚执行过的任务满血复活，将透支的时间从下一个完整周期里扣除
  if (patrol_enabled && s_ran_patrol) {
    s_patrol_left_us += p_period_us;
  }

  // 4. 处理中途通过云端打开的任务，或云端动态缩短了周期的场景
  // 如果剩余时间变成了负数，或者剩余倒计时比当前最新配置的周期还要长，则强制对齐到新周期
  if (patrol_enabled && !s_ran_patrol &&
      (s_patrol_left_us <= 0 || s_patrol_left_us > p_period_us)) {
    s_patrol_left_us = p_period_us;
  }

  // 5. 寻找下一次唤醒需要睡眠的最短倒计时
  int64_t next_sleep_us = -1;
  int64_t min_period_us = 0;

  if (patrol_enabled) {
    next_sleep_us = s_patrol_left_us;
    min_period_us = p_period_us;
  } else {
    next_sleep_us = -1;
    min_period_us = 0;
  }

  if (server_report_task_is_active()) {
    const int64_t server_left_us = server_report_task_left_us();
    if (server_left_us <= 0) {
      next_sleep_us = IMMEDIATE_WAKEUP_US;
    } else if (next_sleep_us < 0 || server_left_us < next_sleep_us) {
      next_sleep_us = server_left_us;
    }
  }

  if (next_sleep_us < 0) {
    return 0; // 全被禁用，且无服务器任务，无限休眠
  }

  // 6. 正常情况下，睡眠时间是周期剩余量：patrol周期 - 实际工作时间。
  // 若本轮工作已经耗尽或超过周期，则放弃已经错过的周期，重新锚定：
  // 完整睡眠一个patrol周期后再开始下一次采集，避免立即连续采集。
  if (next_sleep_us <= 0) {
    next_sleep_us = patrol_enabled ? p_period_us : IMMEDIATE_WAKEUP_US;
    if (patrol_enabled)
      s_patrol_left_us = p_period_us;
  }

  // 7. 防护：防止由于周期动态缩小等边缘配置跳变导致极大的睡眠时间
  if (min_period_us > 0 && next_sleep_us > min_period_us) {
    next_sleep_us = min_period_us;
  }

  return (uint64_t)next_sleep_us;
}

esp_err_t task_daq_trigger_patrol_now(void) {
  s_patrol_left_us = 0;
  return ESP_OK;
}

esp_err_t task_daq_trigger_wom_patrol(void) {
  s_is_wom_wakeup = true;
  return ESP_OK;
}
