#ifndef DAQ_ICM_42688_P_H_
#define DAQ_ICM_42688_P_H_
#include "drv_icm_42688_p.h"

#include <stdint.h>
// Data Acquisition
// 定义业务层用来接收数据的"回调合同"
// data: 捞出来的原始数据块, count: 数据点数, user_ctx: 业务层传进来的上下文(比如 welford 结构体)
typedef void (*daq_data_handler_t)(const imu_raw_data_t *data, size_t count, void *user_ctx);
esp_err_t daq_icm_42688_p_init(void);
// 统一的采集总控入口
esp_err_t daq_icm_42688_p_capture(icm_cfg_t *cfg, 
                          uint32_t duration_ms, 
                          daq_data_handler_t handler, 
                          void *user_ctx, 
                          int16_t chunck_count,
                          uint32_t skip_ms);
#endif // DAQ_ICM_42688_P_H_
