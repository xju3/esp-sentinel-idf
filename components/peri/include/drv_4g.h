#ifndef DRV_4G_H_
#define DRV_4G_H_

#include "esp_err.h"
#include "bsp_network.h"
#include "hal/gpio_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 4G modem GPIO assignments
#define DRV_4G_PIN_PWR          GPIO_NUM_13
#define DRV_4G_PIN_PWRKEY       GPIO_NUM_14
#define DRV_4G_PIN_UART_TX      GPIO_NUM_17
#define DRV_4G_PIN_UART_RX      GPIO_NUM_18
#define DRV_4G_PIN_STATUS       GPIO_NUM_21
#define DRV_4G_PIN_NET_STATUS   GPIO_NUM_47
#define DRV_4G_PIN_RESET_N      GPIO_NUM_48
esp_err_t init_ppp_4g(cb_communication_channel_established cb);
esp_err_t shutdown_ppp_4g(void);
esp_err_t init_4g_network(cb_communication_channel_established cb);
esp_err_t shutdown_4g_network(void);
esp_err_t bsp_4g_http_get(const char *url, char **out_response);

esp_err_t bsp_4g_http_post_json(const char *url, const char *payload, char **out_response);
esp_err_t bsp_4g_http_post_binary(const char *url, const void *payload,
                                  size_t payload_len, char **out_response);
esp_err_t bsp_4g_http_put(const char *url, const char *payload);
esp_err_t bsp_4g_get_rssi(int *out_rssi);

#ifdef __cplusplus
}
#endif

#endif /* DRV_4G_H_ */
