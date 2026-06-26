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
esp_err_t init_4g_network(cb_communication_channel_established cb);
esp_err_t shutdown_4g_network(void);
esp_err_t bsp_4g_http_get(const char *url, char **out_response);

esp_err_t bsp_4g_http_post_json(const char *url, const char *payload, char **out_response);
esp_err_t bsp_4g_http_put(const char *url, const char *payload);
esp_err_t bsp_4g_get_rssi(int *out_rssi);

#ifdef __cplusplus
}
#endif

#endif /* PPP_4G_H_ */
