#ifndef INIT_H
#define INIT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif
    extern bool is_network_available;
    esp_err_t init_nvs();
    esp_err_t start_local_services();
#ifdef __cplusplus
}
#endif

#endif // INIT_H
