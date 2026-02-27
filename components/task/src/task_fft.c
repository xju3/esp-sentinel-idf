#include "drv_icm_42688_p.h"
#include "daq_icm_42688_p.h"
#include "task_fft.h"
#include "task_monitor.h"
#include "logger.h"

#define LSB_TO_G (16.0f / 32768.0f)
#define FFT_CHUNCK_SIZE  128


static void fft_chunk_handler(const imu_raw_data_t *data, size_t count, void *ctx)
{
    fft_capture_ctx_t *fft_ctx = (fft_capture_ctx_t *)ctx;
    
    for (size_t i = 0; i < count; i++) {
        // 防止数组越界爆内存
        if (fft_ctx->write_index >= FFT_POINTS) break; 

        // 解析并存入大数组中保留全尸！
        fft_ctx->x_buffer[fft_ctx->write_index] = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G;
        fft_ctx->y_buffer[fft_ctx->write_index] = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G;
        fft_ctx->z_buffer[fft_ctx->write_index] = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G;
        
        fft_ctx->write_index++;
    }
}


void run_fft_diagnosis(void)
{
    LOG_DEBUG("Starting FFT diagnosis...");
    
    // 1. 暂停监控任务的定时唤醒
    task_monitor_pause_for_fft();
    
    fft_capture_ctx_t fft_ctx;
    fft_ctx.write_index = 0; // 游标清零

    icm_cfg_t cfg = { .odr = ICM_ODR_16KHZ, .fs = ICM_FS_16G };
    daq_icm_42688_p_capture(&cfg, 70, fft_chunk_handler, &fft_ctx, 128);

    if (fft_ctx.write_index >= FFT_POINTS) {
        LOG_DEBUG("FFT Buffer full! Starting DSP calculation...");
        // TODO: 这里添加实际的FFT计算逻辑
    }
    
    // 2. FFT诊断完成，恢复监控任务的定时唤醒
    task_monitor_resume_after_fft();
    
    LOG_DEBUG("FFT diagnosis completed, monitor resumed");
}