#include "task_rms.h"
#include "algo_rms.h"
#include "algo_iso.h"
#include "logger.h"
#include "config_manager.h"
#include "data_dispatcher.h"
#include "esp_timer.h"

// 必须与 daq_worker.c 中的定义保持一致
#define MAX_DAQ_SAMPLES 8192
#define RMS_QUEUE_LEN   5

QueueHandle_t g_rms_job_queue = NULL;

static void rms_task_entry(void *arg)
{
    vib_job_t job;
    while (1)
    {
        // 阻塞等待采集任务发来的数据指针
        if (xQueueReceive(g_rms_job_queue, &job, portMAX_DELAY))
        {
            LOG_INFOF("[TASK_RMS] Received job. Mode: %d, Len: %lu, SR: %.1f", 
                      job.task_mode, job.length, job.sample_rate);

            // 1. 根据平面化布局 (Planar Layout) 获取各轴指针
            // 内存布局: [X0...Xn | Y0...Yn | Z0...Zn]
            // 注意：偏移量固定为 MAX_DAQ_SAMPLES，而非 job.length
            const float *x_ptr = job.raw_data;
            const float *y_ptr = job.raw_data + MAX_DAQ_SAMPLES;
            const float *z_ptr = job.raw_data + MAX_DAQ_SAMPLES * 2;

            // 2. 调用纯算法库计算 RMS (包含 HPF/LPF/积分)
            vib_rms_t rms = algo_rms_calculate(x_ptr, y_ptr, z_ptr, job.length, job.sample_rate);
            
            LOG_INFOF("[TASK_RMS] Result (mm/s): X=%.2f, Y=%.2f, Z=%.2f", rms.x, rms.y, rms.z);

            // 3. 将结果发送给数据分发器 (Data Dispatcher)
            if (g_monitor_message_queue)
            {
                dispatch_msg_t msg = {
                    .type = MSG_TYPE_RMS,
                    .payload.rms = {
                        .timestamp = (uint32_t)(esp_timer_get_time() / 1000), // ms
                        .rms_x = rms.x,
                        .rms_y = rms.y,
                        .rms_z = rms.z
                    }
                };
                xQueueSend(g_monitor_message_queue, &msg, 0);
            }

            // 4. 根据 ISO 10816 标准判断振动状态
            float max_v = (rms.x > rms.y) ? rms.x : rms.y;
            max_v = (max_v > rms.z) ? max_v : rms.z;

            iso_alarm_status_t status = ISO_STATUS_INVALID_CONFIG;
            if (g_user_config.iso.standard == 1) // 1 = ISO10816
            {
                status = iso10816_check(max_v, &g_user_config.iso);
            }
            // else if (g_user_config.iso.standard == 2) { /* TODO: ISO20816 logic */ }

            LOG_INFOF("[TASK_RMS] Vibration status: %s", iso_status_to_string(status));

            // 5. 根据状态触发报警和进一步分析
            if (status >= ISO_STATUS_UNSATISFACTORY)
            {
                if (status == ISO_STATUS_UNACCEPTABLE) {
                    LOG_ERRORF("[TASK_RMS] CRITICAL ALARM! Vibration level is unacceptable. Max: %.2f mm/s", max_v);
                } else {
                    LOG_WARNF("[TASK_RMS] Vibration Alarm! Level is unsatisfactory. Max: %.2f mm/s", max_v);
                }
                // 状态不佳时，触发FFT分析
                LOG_INFO("[TASK_RMS] Triggering advanced analysis (FFT) due to alarm...");
                // TODO: 调用 algo_fft_calculate(x_ptr, job.length, ...);
            }
            else if (job.task_mode == TASK_MODE_DIAGNOSIS)
            {
                // 即使状态良好，诊断模式也需要FFT
                LOG_INFO("[TASK_RMS] Diagnosis mode: Triggering advanced analysis (FFT)...");
                // TODO: 调用 algo_fft_calculate(x_ptr, job.length, ...);
            }
        }
    }
}

void start_rms_diagnosis(void)
{
    if (g_rms_job_queue == NULL)
    {
        g_rms_job_queue = xQueueCreate(RMS_QUEUE_LEN, sizeof(vib_job_t));
    }

    // 创建处理任务
    if (xTaskCreate(rms_task_entry, "rms_task", 4096, NULL, 4, NULL) != pdPASS)
    {
        LOG_ERROR("[TASK_RMS] Failed to create RMS task");
    }
    LOG_INFO("[TASK_RMS] RMS diagnosis task started"); 
}