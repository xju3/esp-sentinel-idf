#ifndef INIT_H
#define INIT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "bsp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif
extern bool is_network_available;
esp_err_t init_nvs();
void enable_config_service();
esp_err_t init_imu_sensors();
esp_err_t init_comm_channel();
esp_err_t enable_mqtt_proxy();
#ifdef __cplusplus
}
#endif

#endif // INIT_H
