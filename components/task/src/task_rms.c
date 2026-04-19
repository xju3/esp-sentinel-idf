#include "task_rms.h"
#include "algo_rms.h"
#include "algo_welford.h"
#include "iso_check.h"
#include "logger.h"
#include "config_manager.h"
#include "off_sleep_manager.h"
#include "task_fft.h"
#include "drv_ds18b20.h"

#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

// 必须与 daq_worker.c 中的定义保持一致
#define MAX_DAQ_SAMPLES 8192
#define RMS_QUEUE_LEN 5
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define FFT_ENABLE_RMS_THRESHOLD_MM_S 0.10f
#define RMS_DEBUG_ANALYSIS_THRESHOLD_MM_S 1.00f

QueueHandle_t g_rms_job_queue = NULL;
static uint32_t s_patrol_low_rms_streak = 0;

static float max_axis_rms_mm_s(const vib_3axis_features_t *features)
{
    if (!features)
    {
        return 0.0f;
    }

    float max_rms = features->x_axis.rms;
    if (features->y_axis.rms > max_rms)
    {
        max_rms = features->y_axis.rms;
    }
    if (features->z_axis.rms > max_rms)
    {
        max_rms = features->z_axis.rms;
    }
    return max_rms;
}

static bool should_run_fft_for_features(const vib_3axis_features_t *features)
{
    if (!features)
    {
        return false;
    }

    float max_rms = max_axis_rms_mm_s(features);
    return max_rms >= FFT_ENABLE_RMS_THRESHOLD_MM_S;
}

#if CONFIG_SENTINEL_ENABLE_OFF_SLEEP_GATE
static bool should_request_off_sleep_mode(const vib_job_t *job, const vib_3axis_features_t *features)
{
    if (!job || !features || job->task_mode != TASK_MODE_PATROLING)
    {
        s_patrol_low_rms_streak = 0;
        return false;
    }

    const float max_rms = max_axis_rms_mm_s(features);
    const float off_threshold = (float)CONFIG_SENTINEL_OFF_RMS_THRESHOLD_MM_S;

    if (max_rms >= off_threshold)
    {
        s_patrol_low_rms_streak = 0;
        return false;
    }

    s_patrol_low_rms_streak++;
    LOG_INFOF("Low patrol RMS detected: max_rms=%.4f mm/s, threshold=%.4f mm/s, streak=%lu/%d",
              max_rms,
              off_threshold,
              (unsigned long)s_patrol_low_rms_streak,
              CONFIG_SENTINEL_OFF_CONFIRM_COUNT);

    if (s_patrol_low_rms_streak < (uint32_t)CONFIG_SENTINEL_OFF_CONFIRM_COUNT)
    {
        return false;
    }

    s_patrol_low_rms_streak = 0;
    LOG_WARN("Machine appears OFF from patrol RMS. Requesting WoM light sleep.");
    return true;
}
#endif

static esp_err_t rms_report(vib_3axis_features_t *features,
                            iso_alarm_status_t status,
                            task_mode_t mode,
                            float sample_rate,
                            float temperature)
{
    (void)sample_rate;

    MsgRmsReport msg_rms_report = MSG_RMS_REPORT__INIT;
    MsgTriaxialValue rms = MSG_TRIAXIAL_VALUE__INIT;
    MsgTriaxialValue peak = MSG_TRIAXIAL_VALUE__INIT;
    MsgTriaxialValue crest = MSG_TRIAXIAL_VALUE__INIT;

    rms.x = features->x_axis.rms;
    rms.y = features->y_axis.rms;
    rms.z = features->z_axis.rms;
    rms.m = vib_3d_norm(features->x_axis.rms, features->y_axis.rms, features->z_axis.rms);
    msg_rms_report.rms = &rms;

    peak.x = features->x_axis.peak;
    peak.y = features->y_axis.peak;
    peak.z = features->z_axis.peak;
    peak.m = vib_3d_norm(features->x_axis.peak, features->y_axis.peak, features->z_axis.peak);
    msg_rms_report.peak = &peak;

    crest.x = features->x_axis.crest_factor;
    crest.y = features->y_axis.crest_factor;
    crest.z = features->z_axis.crest_factor;
    crest.m = vib_3d_norm(features->x_axis.crest_factor, features->y_axis.crest_factor, features->z_axis.crest_factor);
    msg_rms_report.crest = &crest;

    MsgTriaxialValue impulse = MSG_TRIAXIAL_VALUE__INIT;
    impulse.x = features->x_axis.impulse_factor;
    impulse.y = features->y_axis.impulse_factor;
    impulse.z = features->z_axis.impulse_factor;
    impulse.m = vib_3d_norm(features->x_axis.impulse_factor, features->y_axis.impulse_factor, features->z_axis.impulse_factor);
    msg_rms_report.impulse = &impulse;
    msg_rms_report.iso = (int8_t)status;
    msg_rms_report.temperature = temperature;

    (void)mode;
    return send_protobuf_message(1, &msg_rms_report.base);
}

static void log_axis_rms_debug(const char *axis_name, const axis_rms_debug_t *debug)
{
    if (!axis_name || !debug)
    {
        return;
    }

    LOG_INFOF("RMS cause axis %s: total=%.4f mm/s, halves[first=%.4f,second=%.4f], band_est[10-30=%.4f,30-80=%.4f,80-200=%.4f,200-500=%.4f,500-1000=%.4f]",
              axis_name,
              debug->total_rms,
              debug->first_half_rms,
              debug->second_half_rms,
              debug->rms_10_30_hz,
              debug->rms_30_80_hz,
              debug->rms_80_200_hz,
              debug->rms_200_500_hz,
              debug->rms_500_1000_hz);
}

static void log_patrol_rms_debug_if_needed(const vib_job_t *job,
                                           const vib_3axis_features_t *features,
                                           const float *x_ptr,
                                           const float *y_ptr,
                                           const float *z_ptr)
{
    if (!job || !features || !x_ptr || !y_ptr || !z_ptr)
    {
        return;
    }

    if (job->task_mode != TASK_MODE_PATROLING)
    {
        return;
    }

    float max_rms = features->x_axis.rms;
    if (features->y_axis.rms > max_rms)
    {
        max_rms = features->y_axis.rms;
    }
    if (features->z_axis.rms > max_rms)
    {
        max_rms = features->z_axis.rms;
    }
    if (max_rms < RMS_DEBUG_ANALYSIS_THRESHOLD_MM_S)
    {
        return;
    }

    vib_3axis_rms_debug_t debug = {0};
    if (algo_rms_calculate_debug(x_ptr, y_ptr, z_ptr, job->length, job->sample_rate, &debug) != ESP_OK)
    {
        LOG_WARN("Failed to calculate patrol RMS debug bands");
        return;
    }

    LOG_INFOF("Patrol RMS cause debug enabled: max_rms=%.4f mm/s, threshold=%.4f mm/s",
              max_rms,
              RMS_DEBUG_ANALYSIS_THRESHOLD_MM_S);
    log_axis_rms_debug("X", &debug.x_axis);
    log_axis_rms_debug("Y", &debug.y_axis);
    log_axis_rms_debug("Z", &debug.z_axis);
}

/**
 * @brief RMS 任务入口：对三轴振动数据计算 RMS/PEAK/CREST/IMPULSE 等特征并上报
 *
 * 计算方法（对速度信号 v[i]）：
 * - RMS: 均方根，rms = sqrt(mean(v[i]^2))
 * - PEAK: 窗口内最大绝对值，peak = max(|v[i]|)
 * - CREST: 峰值因子，crest = peak / rms（RMS 越小，crest 越大）
 * - IMPULSE: 脉冲指标，impulse = peak / mav，其中 mav = mean(|v[i]|)
 *
 * 物理意义：
 * - RMS 表征整体能量/振动强度
 * - PEAK 反映瞬时最大幅值
 * - CREST 衡量尖峰程度（峰值相对能量）
 * - IMPULSE 衡量冲击性（峰值相对平均幅值）
 */
static void rms_task_entry(void *arg)
{
    vib_job_t job;
    while (1)
    {
        // 阻塞等待采集任务发来的数据指针
        if (xQueueReceive(g_rms_job_queue, &job, portMAX_DELAY))
        {
            LOG_DEBUGF("Received job. Mode: %d, Len: %lu, SR: %.1f",
                       job.task_mode, job.length, job.sample_rate);

            // 1. 根据平面化布局 (Planar Layout) 获取各轴指针
            // 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
            // 注意：偏移量固定为 MAX_DAQ_SAMPLES，而非 job.length
            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;

            // 2. 调用纯算法库计算 RMS (包含 HPF/LPF/积分)
            vib_3axis_features_t features = algo_rms_calculate(x_ptr, y_ptr, z_ptr, job.length, job.sample_rate);
            LOG_DEBUGF("rms(mm/s): X=%.4f, Y=%.4f, Z=%.4f, sample rate=%.4f",
                       features.x_axis.rms,
                       features.y_axis.rms,
                       features.z_axis.rms, job.sample_rate);
            log_patrol_rms_debug_if_needed(&job, &features, x_ptr, y_ptr, z_ptr);

            // 4. 根据 ISO 标准判断振动状态
            float max_v = (features.x_axis.rms > features.y_axis.rms) ? features.x_axis.rms : features.y_axis.rms;
            max_v = (max_v > features.z_axis.rms) ? max_v : features.z_axis.rms;
            iso_alarm_status_t status = ISO_STATUS_INVALID_CONFIG;
            // 使用临时配置，避免修改全局配置
            iso_config_t temp_config = g_user_config.iso;
            // 如果standard为0，使用ISO10816作为默认值
            if (temp_config.standard == 0)
            {
                temp_config.standard = 1; // 默认使用 ISO10816
            }

            if (temp_config.standard == 1) // 1 = ISO10816
            {
                status = iso10816_check(max_v, &temp_config);
            }

            else if (temp_config.standard == 2) // 2 = ISO20816
            {
                status = iso20816_check(max_v, &temp_config);
            }

            LOG_INFOF("vibration status: %s, max: %.2f, ISO: %d",
                      iso_status_to_string(status),
                      max_v,
                      g_user_config.iso.standard);

            switch (status)
            {
            case ISO_STATUS_UNACCEPTABLE:
                /* code */
                LOG_ERRORF("CRITICAL ALARM! Vibration level is unacceptable. Max: %.2f mm/s", max_v);
                break;
            case ISO_STATUS_UNSATISFACTORY:
                LOG_WARNF("Vibration Alarm! Level is unsatisfactory. Max: %.2f mm/s", max_v);
                break;
            default:
                LOG_INFOF("Vibration level is acceptable. Max: %.2f mm/s", max_v);
                break;
            }

            bool request_off_sleep = false;
#if CONFIG_SENTINEL_ENABLE_OFF_SLEEP_GATE
            request_off_sleep = should_request_off_sleep_mode(&job, &features);
#endif

            if (job.task_mode == TASK_MODE_PATROLING && g_fft_job_queue != NULL)
            {
                if (request_off_sleep || off_sleep_manager_sleep_requested())
                {
                    LOG_INFO("Skipping FFT: OFF sleep requested for patrol pipeline");
                }
                else if (should_run_fft_for_features(&features))
                {
                    xQueueSend(g_fft_job_queue, &job, 0);
                }
                else
                {
                    float max_rms = max_axis_rms_mm_s(&features);

                    LOG_INFOF("Skipping FFT: vibration energy too low for patrol analysis (max_rms=%.4f mm/s, threshold=%.4f mm/s)",
                              max_rms,
                              FFT_ENABLE_RMS_THRESHOLD_MM_S);
                }
            }

            // 6. 生成报告并发送到数据分发器
            float temp = 0.0f;
            // 读取温度传感器
            if (g_ds18b20_initialized && drv_ds18b20_read_temperature(&temp) != ESP_OK)
            {
                LOG_ERROR("Failed to read temperature");
            }
            LOG_DEBUGF("Current temperature: %.2f °C", temp);
            rms_report(&features, status, job.task_mode, job.sample_rate, temp);

#if CONFIG_SENTINEL_ENABLE_OFF_SLEEP_GATE
            if (request_off_sleep)
            {
                esp_err_t sleep_req_ret = off_sleep_manager_request_sleep();
                if (sleep_req_ret != ESP_OK)
                {
                    LOG_ERRORF("Failed to request OFF sleep: %s", esp_err_to_name(sleep_req_ret));
                }
            }
#endif
        }
    }
}

esp_err_t start_rms_task(void)
{
    if (g_rms_job_queue == NULL)
    {
        g_rms_job_queue = xQueueCreateWithCaps(RMS_QUEUE_LEN, sizeof(vib_job_t), TASK_MEM_CAPS);
        if (g_rms_job_queue == NULL)
        {
            LOG_ERROR("Failed to create RMS queue");
            return ESP_ERR_NO_MEM;
        }
    }

    // RMS task can enter light sleep through the OFF/WoM path. Keep its stack
    // in internal RAM so the task can resume safely after wake on ESP32-S3.
    if (xTaskCreate(rms_task_entry, "rms_task", 4096, NULL, 4, NULL) != pdPASS)
    {
        LOG_ERROR("Failed to create RMS task");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
