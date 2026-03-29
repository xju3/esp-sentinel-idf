#include "task_rms.h"
#include "algo_rms.h"
#include "algo_welford.h"
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


QueueHandle_t g_rms_job_queue = NULL;
static esp_err_t rms_report(vib_3axis_features_t *features,
                            iso_alarm_status_t status,
                            task_mode_t mode,
                            float sample_rate,
                            float temperature)
{
    (void)mode;
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
    // Send using unified message dispatcher
    return send_protobuf_message(1, &msg_rms_report.base);
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
            float temp = 0.0f;
            // 读取温度传感器
            if (g_ds18b20_initialized && drv_ds18b20_read_temperature(&temp) != ESP_OK)
            {
                LOG_ERROR("Failed to read temperature");
            }
            LOG_DEBUGF("Current temperature: %.2f °C", temp);
            rms_report(&features, status, job.task_mode, job.sample_rate, temp);
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

    // 创建处理任务
    if (xTaskCreateWithCaps(rms_task_entry, "rms_task", 4096, NULL, 4, NULL, TASK_MEM_CAPS) != pdPASS)
    {
        LOG_ERROR("Failed to create RMS task");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
