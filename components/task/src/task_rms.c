#include "task_rms.h"
#include "algo_rms.h"
#include "iso_check.h"
#include "logger.h"
#include "config_manager.h"
#include "data_dispatcher.h"
#include "task_fft.h"
#include "drv_ds18b20.h"

#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
// 必须与 daq_worker.c 中的定义保持一致
#define MAX_DAQ_SAMPLES 8192
#define RMS_QUEUE_LEN 5
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
static task_rms_report_t report = {0} QueueHandle_t g_rms_job_queue = NULL;

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
            vib_3axis_features_t features = algo_rms_calculate_3axis(x_ptr, y_ptr, z_ptr, job.length, job.sample_rate);
            LOG_DEBUGF("rms(mm/s): X=%.2f, Y=%.2f, Z=%.2f, sample rate=%.2f",
                       features.x_axis.rms,
                       features.y_axis.rms,
                       features.z_axis.rms, job.sample_rate);

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

            // 5. 根据状态触发报警和进一步分析
            if (status >= ISO_STATUS_UNSATISFACTORY)
            {
                if (status == ISO_STATUS_UNACCEPTABLE)
                {
                    LOG_ERRORF("CRITICAL ALARM! Vibration level is unacceptable. Max: %.2f mm/s", max_v);
                }
                else
                {
                    LOG_WARNF("Vibration Alarm! Level is unsatisfactory. Max: %.2f mm/s", max_v);
                }
                // 状态不佳时，触发FFT分析
                if (job.task_mode == TASK_MODE_PATROLING)
                {
                    if (g_fft_job_queue != NULL)
                    {
                        xQueueSend(g_fft_job_queue, &job, 0);
                    }
                }
                else
                {
                }
            }

            // 6. 生成报告并发送到数据分发器
            float temp = ds18b20_read_temperature(); // 读取温度传感器
            report(job.task_id, &features, status, job.task_mode, job.sample_rate, (temp * 100));
        }
    }
}

esp_err_t report(int32_t task_id,
                 vib_3axis_features_t *features,
                 iso_alarm_status_t status,
                 task_mode_t mode,
                 float sample_rate,
                 float temperature)
{

    if (g_msg_dispatcher_queue == NULL)
    {
        LOG_ERROR("Message dispatcher queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    report.task_id = task_id; // 可根据实际情况设置
    report.timestamp = time(NULL);
    report.sample_rate = sample_rate; // 可根据实际情况设置
    report.temperature = temperature; // 可根据实际情况设置
    report.conclusion = (int8_t)status;
    report.task_mode = mode;
    report.x_axis.rms = features->x_axis.rms;
    report.y_axis.rms = features->y_axis.rms;
    report.z_axis.rms = features->z_axis.rms;
    report.x_axis.peak = features->x_axis.peak;
    report.y_axis.peak = features->y_axis.peak;
    report.z_axis.peak = features->z_axis.peak;
    report.x_axis.crest_factor = features->x_axis.crest_factor;
    report.y_axis.crest_factor = features->y_axis.crest_factor;
    report.z_axis.crest_factor = features->z_axis.crest_factor;
    report.x_axis.impulse_factor = features->x_axis.impulse_factor;
    report.y_axis.impulse_factor = features->y_axis.impulse_factor;
    report.z_axis.impulse_factor = features->z_axis.impulse_factor;
    g_binary_msg.data = (uint8_t *)&report;
    g_binary_msg.len = sizeof(report);
    strncpy(g_binary_msg.topic, "device/vibration/rms", sizeof(g_binary_msg.topic) - 1);
    g_binary_msg.topic[sizeof(g_binary_msg.topic) - 1] = '\0';

    xQueueSend(g_msg_dispatcher_queue, &g_binary_msg, 0);
    return ESP_OK;
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

    // 创建处理任务
    if (xTaskCreateWithCaps(rms_task_entry, "rms_task", 4096, NULL, 4, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create RMS task");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
