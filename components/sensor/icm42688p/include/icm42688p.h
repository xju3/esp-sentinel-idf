#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Override these at build time or via sdkconfig if desired.
#ifdef CONFIG_ICM42688P_SPI_HOST_ID
#define ICM42688P_SPI_HOST CONFIG_ICM42688P_SPI_HOST_ID
#else
#define ICM42688P_SPI_HOST SPI2_HOST
#endif

#ifdef CONFIG_ICM42688P_SPI_MISO
#define ICM42688P_SPI_MISO CONFIG_ICM42688P_SPI_MISO
#else
#define ICM42688P_SPI_MISO 13
#endif

#ifdef CONFIG_ICM42688P_SPI_MOSI
#define ICM42688P_SPI_MOSI CONFIG_ICM42688P_SPI_MOSI
#else
#define ICM42688P_SPI_MOSI 11
#endif

#ifdef CONFIG_ICM42688P_SPI_SCK
#define ICM42688P_SPI_SCK CONFIG_ICM42688P_SPI_SCK
#else
#define ICM42688P_SPI_SCK 12
#endif

#ifdef CONFIG_ICM42688P_SPI_CS
#define ICM42688P_SPI_CS CONFIG_ICM42688P_SPI_CS
#else
#define ICM42688P_SPI_CS 10
#endif

typedef struct {
    float x_g;
    float y_g;
    float z_g;
} icm42688p_sample_t;

// Initialize SPI bus + device and wake the sensor.
esp_err_t icm42688p_init(void);

// Read accelerometer in g units.
esp_err_t icm42688p_read_accel(icm42688p_sample_t *out_sample);

// Read WHO_AM_I register; returns ESP_OK and fills id.
esp_err_t icm42688p_who_am_i(uint8_t *out_id);

#ifdef __cplusplus
}
#endif

#endif // ICM42688P_H
