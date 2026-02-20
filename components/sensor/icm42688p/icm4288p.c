// Minimal SPI driver for ICM-42688-P on ESP-IDF, modeled after Arduino HAL.
#include "icm4288p.h"
#include "logger.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ICM42688P_REG_WHO_AM_I   0x75
#define ICM42688P_REG_PWR_MGMT0  0x4E
#define ICM42688P_REG_ACCEL_X1   0x1F

#define ICM42688P_SPI_HZ 10 * 1000 * 1000
#define ICM42688P_ACCEL_LSB_PER_G 2048.0f

static spi_device_handle_t s_icm_dev = NULL;

static esp_err_t icm_spi_init_bus(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ICM4288P_SPI_MOSI,
        .miso_io_num = ICM4288P_SPI_MISO,
        .sclk_io_num = ICM4288P_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    esp_err_t err = spi_bus_initialize(ICM4288P_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        // Bus already initialized; accept.
        return ESP_OK;
    }
    return err;
}

static esp_err_t icm_spi_add_device(void)
{
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ICM42688P_SPI_HZ,
        .mode = 0,
        .spics_io_num = ICM4288P_SPI_CS,
        .queue_size = 2,
        // Use full-duplex; half-duplex forbids simultaneous MOSI/MISO phases we need for burst reads.
        .flags = 0,
    };
    return spi_bus_add_device(ICM4288P_SPI_HOST, &dev_cfg, &s_icm_dev);
}

static esp_err_t icm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(s_icm_dev, &t);
}

static esp_err_t icm_read_reg(uint8_t reg, uint8_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .flags = 0,
    };
    esp_err_t err = spi_device_polling_transmit(s_icm_dev, &t);
    if (err == ESP_OK) {
        *out = rx[1];
    }
    return err;
}

static esp_err_t icm_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (!ax || !ay || !az) return ESP_ERR_INVALID_ARG;

    uint8_t tx[7] = { (uint8_t)(ICM42688P_REG_ACCEL_X1 | 0x80), 0,0,0,0,0,0 };
    uint8_t rx[7] = {0};
    spi_transaction_t t = {
        .length = 7 * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .flags = 0,
    };
    esp_err_t err = spi_device_polling_transmit(s_icm_dev, &t);
    if (err != ESP_OK) return err;

    *ax = (int16_t)((rx[1] << 8) | rx[2]);
    *ay = (int16_t)((rx[3] << 8) | rx[4]);
    *az = (int16_t)((rx[5] << 8) | rx[6]);
    return ESP_OK;
}

esp_err_t icm4288p_init(void)
{
    esp_err_t err = icm_spi_init_bus();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        LOG_ERRORF("ICM SPI bus init failed: %d", err);
        return err;
    }

    err = icm_spi_add_device();
    if (err != ESP_OK) {
        LOG_ERRORF("ICM add device failed: %d", err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t id = 0;
    err = icm_read_reg(ICM42688P_REG_WHO_AM_I, &id);
    if (err != ESP_OK) {
        LOG_ERRORF("ICM read WHO_AM_I failed: %d", err);
        return err;
    }
    if (id == 0x00 || id == 0xFF) {
        LOG_ERROR("ICM invalid WHO_AM_I");
        return ESP_FAIL;
    }

    err = icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x0F);
    if (err != ESP_OK) {
        LOG_ERRORF("ICM write PWR_MGMT0 failed: %d", err);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t icm4288p_who_am_i(uint8_t *out_id)
{
    return icm_read_reg(ICM42688P_REG_WHO_AM_I, out_id);
}

esp_err_t icm4288p_read_accel(icm4288p_sample_t *out_sample)
{
    if (!out_sample) return ESP_ERR_INVALID_ARG;
    int16_t ax = 0, ay = 0, az = 0;
    esp_err_t err = icm_read_accel_raw(&ax, &ay, &az);
    if (err != ESP_OK) return err;

    out_sample->x_g = (float)ax / ICM42688P_ACCEL_LSB_PER_G;
    out_sample->y_g = (float)ay / ICM42688P_ACCEL_LSB_PER_G;
    out_sample->z_g = (float)az / ICM42688P_ACCEL_LSB_PER_G;
    return ESP_OK;
}
