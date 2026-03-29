#ifndef BSP_NETWORK_H
#define BSP_NETWORK_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*cb_wifi_connected)(void);
/**
 * @brief Initialize the network
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on failure
 */
esp_err_t bsp_network_init(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_NETWORK_H
