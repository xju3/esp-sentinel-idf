#ifndef SUPERCAP_MANAGER_H
#define SUPERCAP_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_CHARGE_RECHECK_INTERVAL_MS 1000U
#define BOARD_CHARGE_MAX_HOLD_MS 60000U

esp_err_t supercap_prepare_for_4g(bool need_4g, bool *ready);

#ifdef __cplusplus
}
#endif

#endif // SUPERCAP_MANAGER_H
