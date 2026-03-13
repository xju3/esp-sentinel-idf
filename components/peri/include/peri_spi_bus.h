#ifndef PERI_SPI_BUS_H
#define PERI_SPI_BUS_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the shared SPI2 bus for peri devices.
 * This is idempotent and safe to call multiple times.
 */
esp_err_t peri_spi_bus_init(void);

/**
 * Deinitialize the shared SPI bus. Optional; provided for completeness.
 */
esp_err_t peri_spi_bus_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PERI_SPI_BUS_H
