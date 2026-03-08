#ifndef WOM_LIS2DH12_H
#define WOM_LIS2DH12_H

#include "esp_err.h"
#include "drv_lis2dh12.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the LIS2DH12 Wake-on-Motion listener task.
 *
 * This task initializes the GPIOs for INT1 and INT2 from the LIS2DH12
 * and waits for interrupts, logging a message when one occurs.
 *
 * @return esp_err_t 
 *      - ESP_OK on success
 *      - Other error codes on failure
 */
esp_err_t start_wom_lis2dh12_listener();

/**
 * @brief Manual check for WoM triggers after sleep wakeup.
 * Call this immediately after esp_light_sleep_start() returns.
 */
void wom_lis2dh12_on_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif // WOM_LIS2DH12_H
