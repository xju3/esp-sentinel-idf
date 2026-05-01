#ifndef BOOT_FLOW_H
#define BOOT_FLOW_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t boot_flow_prepare(bool *ready_to_start_local_services);

#ifdef __cplusplus
}
#endif

#endif // BOOT_FLOW_H
