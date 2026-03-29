#ifndef DAQ_COMMON_H_
#define DAQ_COMMON_H_

#include <stddef.h>
#include "imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*daq_data_handler_t)(const imu_raw_data_t *data, size_t count, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif // DAQ_COMMON_H_
