#ifndef WOM_LIS2DH12_H
#define WOM_LIS2DH12_H

#include "esp_err.h"
#include "drv_lis2dh12.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the LIS2DH12 Wake-on-Motion listener infrastructure.
 *
 * This prepares the listener task and GPIO interrupt service, but does not
 * arm WoM on the sensor until wom_lis2dh12_enable() is called.
 *
 * @return esp_err_t 
 *      - ESP_OK on success
 *      - Other error codes on failure
 */
esp_err_t start_wom_lis2dh12_listener();

esp_err_t wom_lis2dh12_enable(void);
esp_err_t wom_lis2dh12_disable(void);
esp_err_t wom_lis2dh12_enter_light_sleep_until_wakeup(void);

/**
 * @brief Manual check for WoM triggers after sleep wakeup.
 * Call this immediately after esp_light_sleep_start() returns.
 */
void wom_lis2dh12_on_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif // WOM_LIS2DH12_H
