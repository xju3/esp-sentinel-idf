#ifndef PPP_4G_H_
#define PPP_4G_H_

#include "esp_err.h"
#include "esp_event.h"
#include "bsp_network.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t init_ppp_4g(cb_communication_channel_established cb);
esp_err_t shutdown_ppp_4g(void);
esp_err_t init_4g_mqtt(cb_communication_channel_established cb);
esp_err_t bsp_4g_mqtt_disconnect(void);
esp_err_t shutdown_4g_mqtt(void);
esp_err_t bsp_4g_mqtt_publish(const char *topic, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* PPP_4G_H_ */
