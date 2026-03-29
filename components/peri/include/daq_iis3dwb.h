#ifndef DAQ_IIS3DWB_H_
#define DAQ_IIS3DWB_H_

#include <stdint.h>
#include "daq_common.h"
#include "drv_iis3dwb.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t daq_iis3dwb_init(void);

esp_err_t daq_iis3dwb_capture(
    iis3dwb_cfg_t *cfg,
    uint32_t duration_ms,
    daq_data_handler_t handler,
    void *user_ctx,
    int16_t chunck_size,
    uint32_t skip_ms
);

#ifdef __cplusplus
}
#endif

#endif // DAQ_IIS3DWB_H_
