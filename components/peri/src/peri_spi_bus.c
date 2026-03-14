#include "peri_spi_bus.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "logger.h"
#include "esp_err.h"

// Silence unused include warnings when building unit tests / static analysis
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_9


static bool s_inited = false;

esp_err_t peri_spi_bus_init(void) {
    if (s_inited) return ESP_OK;

    // Use canonical pin macros where possible; prefer ICM defines for MISO/MOSI/CLK
    spi_bus_config_t buscfg = {
        .miso_io_num    = PIN_NUM_MISO,
        .mosi_io_num    = PIN_NUM_MOSI,
        .sclk_io_num    = PIN_NUM_CLK,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz = 4092,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        LOG_ERRORF("peri_spi_bus_init: spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    s_inited = true;
    LOG_DEBUG("Shared SPI2 bus initialized");
    return ESP_OK;
}

esp_err_t peri_spi_bus_deinit(void) {
    if (!s_inited) return ESP_OK;
    esp_err_t ret = spi_bus_free(SPI2_HOST);
    if (ret == ESP_OK) s_inited = false;
    return ret;
}
