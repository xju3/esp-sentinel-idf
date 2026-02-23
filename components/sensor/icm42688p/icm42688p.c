// Minimal SPI driver for ICM-42688-P on ESP-IDF, modeled after Arduino HAL.
#include "icm42688p.h"
#include "logger.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <stdlib.h>

#define ICM42688P_REG_WHO_AM_I   0x75
#define ICM42688P_REG_PWR_MGMT0  0x4E
#define ICM42688P_REG_ACCEL_X1   0x1F
#define ICM42688P_REG_FIFO_CONFIG  0x16
#define ICM42688P_REG_FIFO_COUNTH  0x2E
#define ICM42688P_REG_FIFO_DATA    0x30
#define ICM42688P_REG_ACCEL_CONFIG0 0x50
#define ICM42688P_REG_FIFO_CONFIG1 0x5F

#define ICM42688P_SPI_HZ 10 * 1000 * 1000
#define ICM42688P_ACCEL_LSB_PER_G 2048.0f

static spi_device_handle_t s_icm_dev = NULL;
static int16_t s_last_ax = 0, s_last_ay = 0, s_last_az = 0;
static int64_t s_last_read_us = -1;
static uint32_t s_last_interval_us = 0;

static esp_err_t icm_spi_init_bus(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ICM42688P_SPI_MOSI,
        .miso_io_num = ICM42688P_SPI_MISO,
        .sclk_io_num = ICM42688P_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092, // Increased to support FIFO burst reads
    };
    esp_err_t err = spi_bus_initialize(ICM42688P_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
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
        .spics_io_num = ICM42688P_SPI_CS,
        .queue_size = 2,
        // Use full-duplex; half-duplex forbids simultaneous MOSI/MISO phases we need for burst reads.
        .flags = 0,
    };
    return spi_bus_add_device(ICM42688P_SPI_HOST, &dev_cfg, &s_icm_dev);
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

// Cached/raw read with user-specified min interval (us). If interval_us==0, always re-read.
esp_err_t icm42688p_read_accel_raw_rate(int16_t *ax, int16_t *ay, int16_t *az, uint32_t interval_us)
{
    if (!ax || !ay || !az) return ESP_ERR_INVALID_ARG;

    int64_t now = esp_timer_get_time();
    bool reuse = (interval_us > 0) &&
                 (s_last_read_us >= 0) &&
                 (now - s_last_read_us < (int64_t)interval_us);

    if (!reuse) {
        esp_err_t err = icm_read_accel_raw(&s_last_ax, &s_last_ay, &s_last_az);
        if (err != ESP_OK) return err;
        s_last_read_us = now;
        s_last_interval_us = interval_us;
    }

    *ax = s_last_ax;
    *ay = s_last_ay;
    *az = s_last_az;
    return ESP_OK;
}

esp_err_t icm42688p_init(void)
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

    // Set ODR to 4kHz (0x04) and FS to +/- 16g (0x00) -> 0x04
    err = icm_write_reg(ICM42688P_REG_ACCEL_CONFIG0, 0x04);
    if (err != ESP_OK) return err;

    // Enable Accel in FIFO (Packet 3: Header + Accel + Temp = 8 bytes)
    // Bit 0: FIFO_ACCEL_EN
    err = icm_write_reg(ICM42688P_REG_FIFO_CONFIG1, 0x01);
    if (err != ESP_OK) return err;

    // Set FIFO mode to Stream-to-FIFO
    // Bits 7:6 = 01 (Stream mode)
    err = icm_write_reg(ICM42688P_REG_FIFO_CONFIG, 0x40);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t icm42688p_who_am_i(uint8_t *out_id)
{
    return icm_read_reg(ICM42688P_REG_WHO_AM_I, out_id);
}

esp_err_t icm42688p_read_accel(icm42688p_sample_t *out_sample)
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

esp_err_t icm42688p_read_fifo(icm42688p_sample_t *out_samples, size_t max_samples, size_t *out_count)
{
    if (!out_samples || !out_count) return ESP_ERR_INVALID_ARG;
    *out_count = 0;

    // 1. Read FIFO Count
    uint8_t tx_cnt[3] = { (uint8_t)(ICM42688P_REG_FIFO_COUNTH | 0x80), 0, 0 };
    uint8_t rx_cnt[3] = {0};
    spi_transaction_t t_cnt = {
        .length = 24,
        .tx_buffer = tx_cnt,
        .rx_buffer = rx_cnt,
    };
    esp_err_t err = spi_device_polling_transmit(s_icm_dev, &t_cnt);
    if (err != ESP_OK) return err;

    uint16_t bytes_in_fifo = (uint16_t)((rx_cnt[1] << 8) | rx_cnt[2]);
    if (bytes_in_fifo == 0) return ESP_OK;

    // Packet 3 size: Header(1) + Accel(6) + Temp(1) = 8 bytes
    const size_t packet_size = 8;
    size_t num_packets = bytes_in_fifo / packet_size;
    if (num_packets > max_samples) num_packets = max_samples;
    if (num_packets == 0) return ESP_OK;

    size_t read_len = num_packets * packet_size;

    // 2. Burst Read FIFO Data (DMA capable buffer required)
    uint8_t *raw_data = heap_caps_malloc(read_len + 1, MALLOC_CAP_DMA);
    if (!raw_data) return ESP_ERR_NO_MEM;

    // First byte is command
    raw_data[0] = (uint8_t)(ICM42688P_REG_FIFO_DATA | 0x80);

    spi_transaction_t t_data = {
        .length = (read_len + 1) * 8,
        .tx_buffer = raw_data,
        .rx_buffer = raw_data, // Read back into same buffer
    };

    // Use interrupt/DMA based transmit to avoid blocking CPU during large transfer
    err = spi_device_transmit(s_icm_dev, &t_data);

    if (err == ESP_OK) {
        for (size_t i = 0; i < num_packets; i++) {
            // Offset 1 for command byte, then i * packet_size
            uint8_t *pkt = &raw_data[1 + i * packet_size];
            
            // Packet 3: Header(1), AccelX(2), AccelY(2), AccelZ(2), Temp(1)
            int16_t ax = (int16_t)((pkt[1] << 8) | pkt[2]);
            int16_t ay = (int16_t)((pkt[3] << 8) | pkt[4]);
            int16_t az = (int16_t)((pkt[5] << 8) | pkt[6]);

            out_samples[i].x_g = (float)ax / ICM42688P_ACCEL_LSB_PER_G;
            out_samples[i].y_g = (float)ay / ICM42688P_ACCEL_LSB_PER_G;
            out_samples[i].z_g = (float)az / ICM42688P_ACCEL_LSB_PER_G;
        }
        *out_count = num_packets;
    }

    free(raw_data);
    return err;
}

esp_err_t icm42688p_set_sleep(bool sleep)
{
    uint8_t pwr_mgmt0_val = sleep ? 0x00 : 0x0F; // 0x00 = sleep, 0x0F = wake (accel+gyro on)
    esp_err_t err = icm_write_reg(ICM42688P_REG_PWR_MGMT0, pwr_mgmt0_val);
    if (err != ESP_OK) {
        LOG_ERRORF("Failed to set sleep mode %d: %d", sleep, err);
        return err;
    }
    
    // Small delay for mode transition
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return ESP_OK;
}
