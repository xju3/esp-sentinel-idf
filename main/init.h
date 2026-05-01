#ifndef INIT_H
#define INIT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "bsp_wifi.h"

#ifdef __cplusplus
extern "C"
{
#endif
    extern bool is_network_available;
    esp_err_t init_nvs();
    esp_err_t log_soc_temperature_once(void);
    esp_err_t start_local_services();
    esp_err_t start_network_services();
    esp_err_t establish_communication_channel();
    void enable_config_service();
    esp_err_t disable_config_service(void);
#ifdef __cplusplus
}
#endif

#endif // INIT_H
