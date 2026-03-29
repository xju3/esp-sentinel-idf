#ifndef DAQ_ICM_42688_P_H_
#define DAQ_ICM_42688_P_H_

#include <stdint.h>
#include "daq_common.h"
#include "drv_icm_42688_p.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t daq_icm_42688_p_init(void);

// 统一的采集总控入口 (自带阻塞等待与分块输出机制)
esp_err_t daq_icm_42688_p_capture(
    icm_cfg_t *cfg, 
    uint32_t duration_ms, 
    daq_data_handler_t handler, 
    void *user_ctx, 
    int16_t chunck_size,
    uint32_t skip_ms
);

#ifdef __cplusplus
}
#endif

#endif // DAQ_ICM_42688_P_H_
