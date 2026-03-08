/**
 * @file drv_lis2dh12.c
 * @brief Driver for LIS2DH12 3-axis accelerometer
 * @version 1.1
 * @date 2024-06-01
 * 
 * Refactored for better readability and robustness.
 */
#include "drv_lis2dh12.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "LIS2DH12";

// Register map
#define LIS2DH12_REG_WHO_AM_I       0x0F
#define LIS2DH12_REG_CTRL_REG1      0x20
#define LIS2DH12_REG_CTRL_REG2      0x21
#define LIS2DH12_REG_CTRL_REG3      0x22
#define LIS2DH12_REG_CTRL_REG4      0x23
#define LIS2DH12_REG_CTRL_REG5      0x24
#define LIS2DH12_REG_CTRL_REG6      0x25
#define LIS2DH12_REG_REFERENCE      0x26
#define LIS2DH12_REG_OUT_X_L        0x28
#define LIS2DH12_REG_INT1_CFG       0x30
#define LIS2DH12_REG_INT1_SRC       0x31
#define LIS2DH12_REG_INT1_THS       0x32
#define LIS2DH12_REG_INT1_DURATION  0x33
#define LIS2DH12_REG_INT2_CFG       0x34
#define LIS2DH12_REG_INT2_SRC       0x35
#define LIS2DH12_REG_INT2_THS       0x36
#define LIS2DH12_REG_INT2_DURATION  0x37

#define LIS2DH12_WHO_AM_I_VAL       0x33

// Bit masks and values
#define LIS2DH12_ODR_50HZ           0x40
#define LIS2DH12_ODR_100HZ          0x50
#define LIS2DH12_XYZ_EN             0x07

#define LIS2DH12_HPIS1              0x01
#define LIS2DH12_HPIS2              0x02

#define LIS2DH12_I1_IA1             0x40
#define LIS2DH12_I2_IA2             0x20 // Correct bit for I2 on INT2 pin

#define LIS2DH12_BDU                0x80
#define LIS2DH12_FS_MASK            0x30

#define LIS2DH12_LIR_INT1           0x08
#define LIS2DH12_LIR_INT2           0x02

#define LIS2DH12_INT_6D             0x40
#define LIS2DH12_INT_AOI            0x80
#define LIS2DH12_INT_XHIE           0x02
#define LIS2DH12_INT_YHIE           0x08
#define LIS2DH12_INT_ZHIE           0x20

// SPI configuration
#define SPI_HOST SPI2_HOST

// --- Driver state ---
static spi_device_handle_t s_spi_handle = NULL;
static SemaphoreHandle_t s_spi_mutex = NULL;
static bool s_initialized = false;
static float s_current_odr = 0.0f;
static lis2dh12_fs_t s_current_fs = LIS2DH12_FS_2G;

// --- Private Functions (Forward Declaration) ---
static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data);
static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data);
static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len);

// --- Private Functions ---

static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length = 8,
        .addr = reg & 0x7F, // MSB=0 for write
        .tx_buffer = &data
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length = 8,
        .addr = reg | 0x80, // MSB=1 for read
        .rx_buffer = data
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length = len * 8,
        .addr = reg | 0x80 | 0x40, // MSB=1 for read, MS=1 for auto-increment
        .rx_buffer = buffer
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}


// ODR mapping
typedef struct {
    float odr_hz;
    uint8_t reg_value;
} lis2dh12_odr_map_t;

static const lis2dh12_odr_map_t s_odr_table[] = {
    {1.0f,    0x01}, {10.0f,   0x02}, {25.0f,   0x03}, {50.0f,   0x04},
    {100.0f,  0x05}, {200.0f,  0x06}, {400.0f,  0x07}, {1344.0f, 0x09} // 1.6kHz is low-power only
};
static const int ODR_TABLE_SIZE = sizeof(s_odr_table) / sizeof(s_odr_table[0]);

static float lis2dh12_config_odr_callback(float ideal_odr) {
    float selected_odr = s_odr_table[ODR_TABLE_SIZE - 1].odr_hz;
    uint8_t selected_reg = s_odr_table[ODR_TABLE_SIZE - 1].reg_value;

    // Find the first ODR that is >= ideal_odr
    for (int i = 0; i < ODR_TABLE_SIZE; i++) {
        if (s_odr_table[i].odr_hz >= ideal_odr) {
            selected_odr = s_odr_table[i].odr_hz;
            selected_reg = s_odr_table[i].reg_value;
            break;
        }
    }
    
    s_current_odr = selected_odr;

    // Write ODR, enable X/Y/Z axes
    uint8_t ctrl_reg1_val = (selected_reg << 4) | 0x07;
    if (lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, ctrl_reg1_val) != ESP_OK) {
        return 0.0f; // Indicate failure
    }

    return selected_odr;
}

// SensorDriver struct for imu_config
SensorDriver_t lis2dh12_driver = {
    .name = "LIS2DH12",
    .config_hardware_odr = lis2dh12_config_odr_callback
};

// --- Public Functions ---
esp_err_t drv_lis2dh12_init(void) {

    if (s_initialized) {
        return ESP_OK;
    }

    s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = LIS2DH12_PIN_NUM_SDO,
        .mosi_io_num = LIS2DH12_PIN_NUM_SDA,
        .sclk_io_num = LIS2DH12_PIN_NUM_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized, skipping init");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (reduced from 5MHz for signal integrity)
        .mode = 0,
        .spics_io_num = LIS2DH12_PIN_NUM_CS,
        .queue_size = 7,
        .address_bits = 8,
    };

    ret = spi_bus_add_device(SPI_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check WHO_AM_I
    uint8_t who_am_i = 0;
    ret = lis2dh12_read_reg(LIS2DH12_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK || who_am_i != LIS2DH12_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I check failed. Got 0x%02x, expected 0x%02x", who_am_i, LIS2DH12_WHO_AM_I_VAL);
        ESP_LOGE(TAG, "SPI communication error or LIS2DH12 not responding.");
        ESP_LOGE(TAG, "Please verify:");
        ESP_LOGE(TAG, "  1. Hardware connections (SCL=%d, MOSI=%d, MISO=%d, CS=%d)", 
                 LIS2DH12_PIN_NUM_SCL, LIS2DH12_PIN_NUM_SDA, LIS2DH12_PIN_NUM_SDO, LIS2DH12_PIN_NUM_CS);
        ESP_LOGE(TAG, "  2. LIS2DH12 power supply");
        ESP_LOGE(TAG, "  3. Correct SPI clock frequency (currently 1MHz)");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Basic configuration
    // Enable BDU, set default FS
    uint8_t ctrl_reg4_val = (s_current_fs << 4) | 0x80;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CTRL_REG4");
        return ret;
    }


    ESP_LOGI(TAG, "LIS2DH12 initialized successfully");
    s_initialized = true;
    return ESP_OK;
}

esp_err_t drv_lis2dh12_set_config(lis2dh12_fs_t fs, float odr) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    // Set ODR
    lis2dh12_config_odr_callback(odr);

    // Set Full-Scale
    s_current_fs = fs;
    uint8_t ctrl_reg4_val;
    lis2dh12_read_reg(LIS2DH12_REG_CTRL_REG4, &ctrl_reg4_val);
    ctrl_reg4_val = (ctrl_reg4_val & 0xCF) | (fs << 4);
    return lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
}

esp_err_t drv_lis2dh12_get_raw_data(lis2dh12_raw_data_t *data) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!data) return ESP_ERR_INVALID_ARG;

    // Read 6 bytes starting from OUT_X_L
    uint8_t buffer[6];
    esp_err_t ret = lis2dh12_read_multiple(LIS2DH12_REG_OUT_X_L, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // Data is LSB-first
    data->header = 0xAA;
    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    data->temp = 0;

    return ESP_OK;
}

lis2dh12_fs_t drv_lis2dh12_get_current_fs(void) {
    return s_current_fs;
}



// Read INT1 source register to clear latched interrupt
esp_err_t drv_lis2dh12_read_int1_source(uint8_t *source) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!source) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT1_SRC, source);
}

// Read INT2 source register to clear latched interrupt
esp_err_t drv_lis2dh12_read_int2_source(uint8_t *source) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!source) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT2_SRC, source);
}

esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg) {

    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!wom_cfg) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Enabling WoM mode...");

    // 1. Set ODR to a low-power friendly rate, e.g., 50Hz, and enter normal mode.
    uint8_t odr_reg_val = LIS2DH12_ODR_50HZ | LIS2DH12_XYZ_EN; // 0x47
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, odr_reg_val));
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for device to stabilize

    // 2. Initialize HPF baseline by reading current acceleration data
    // This establishes the reference point for detecting posture deviation
    lis2dh12_raw_data_t baseline_sample;
    for (int i = 0; i < 5; i++) {
        drv_lis2dh12_get_raw_data(&baseline_sample);
        vTaskDelay(pdMS_TO_TICKS(20)); // Wait for stable reading
    }
    ESP_LOGI(TAG, "Baseline initialized: X=%d, Y=%d, Z=%d", 
             baseline_sample.x, baseline_sample.y, baseline_sample.z);

    // 3. Configure High-Pass filter (HPF)
    // INT1 (Vibration): Enable HPF (HP_IA1=1) to remove gravity and detect shock/shake.
    // INT2 (Posture): Disable HPF (HP_IA2=0) to use static gravity for 6D orientation.
    // CTRL_REG2: HPCF=00 (default), HP_IA2=0, HP_IA1=1 -> 0x01
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, LIS2DH12_HPIS1));
    vTaskDelay(pdMS_TO_TICKS(30)); // Wait for HPF to stabilize

    // 4. Configure INT1 and INT2 routing
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, LIS2DH12_I1_IA1)); // I1_IA1 on INT1
    // Note: Previous code used 0x40 (I2_IA1) for INT2 pin, which routed INT1 engine to INT2 pin.
    // Corrected to 0x20 (I2_IA2) to route INT2 engine to INT2 pin.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, LIS2DH12_I2_IA2)); 

    // 5. Set Thresholds. The value is dependent on the Full-Scale selection.
    // LSB value changes with FS.
    uint16_t sensitivity_mg_lsb = 16; // Default to 2G
    switch (s_current_fs) {
        case LIS2DH12_FS_2G:  sensitivity_mg_lsb = 16;  break;
        case LIS2DH12_FS_4G:  sensitivity_mg_lsb = 32;  break;
        case LIS2DH12_FS_8G:  sensitivity_mg_lsb = 62;  break;
        case LIS2DH12_FS_16G: sensitivity_mg_lsb = 186; break;
    }
    uint8_t thr1 = wom_cfg->threshold_mg_int1 / sensitivity_mg_lsb;
    uint8_t thr2 = wom_cfg->threshold_mg_int2 / sensitivity_mg_lsb;
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_THS, thr1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_THS, thr2));
    ESP_LOGI(TAG, "INT1 threshold: %d LSBs (~%dmg), INT2 threshold: %d LSBs (~%dmg)", 
             thr1, thr1 * sensitivity_mg_lsb, thr2, thr2 * sensitivity_mg_lsb);

    // 6. Set Durations. Duration = value / ODR (50Hz, so 20ms per LSB).
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_DURATION, wom_cfg->duration_int1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_DURATION, wom_cfg->duration_int2));
    ESP_LOGI(TAG, "INT1 duration: %dms, INT2 duration: %dms", 
             wom_cfg->duration_int1 * 20, wom_cfg->duration_int2 * 20);

    // 7. Configure interrupt event logic.
    // INT1: Vibration (Shock). Enable High events only for rapid motion detection
    // AOI=0 (OR logic), XHIE=1, YHIE=1, ZHIE=1
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 
        LIS2DH12_INT_XHIE | LIS2DH12_INT_YHIE | LIS2DH12_INT_ZHIE)); // 0x2A

    // INT2: Posture (6D Movement).
    // AOI=0 (OR logic), 6D=1 (6D detection), Enable all 6 directions (XH, XL, YH, YL, ZH, ZL)
    // Value: 0111 1111 = 0x7F
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x7F)); // 0x7F enables all 6D
    
    // 8. Latch interrupts on INT1 and INT2 to read the source register
    // This prevents interrupt from being cleared until source register is read
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 
        LIS2DH12_LIR_INT1 | LIS2DH12_LIR_INT2)); // 0x0A
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "WoM mode enabled successfully");
    ESP_LOGI(TAG, "INT1: Vibration detection (>%dmg absolute), INT2: Posture detection (6D, >%dmg deviation)", 
             wom_cfg->threshold_mg_int1, wom_cfg->threshold_mg_int2);

    // 9. Clear any pending interrupts to ensure lines are low before we start
    // This is crucial because LIR (Latch) is enabled. If an interrupt triggered during config, it stays high.
    uint8_t dummy;
    lis2dh12_read_reg(LIS2DH12_REG_REFERENCE, &dummy); // Reset HPF filter
    lis2dh12_read_reg(LIS2DH12_REG_INT1_SRC, &dummy);  // Clear INT1
    lis2dh12_read_reg(LIS2DH12_REG_INT2_SRC, &dummy);  // Clear INT2

    return ESP_OK;
}

esp_err_t drv_lis2dh12_disable_wom(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Disabling WoM mode...");

    // Power down the device to reset states
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(5));

    // Disable interrupt generators and routing
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x00));
    
    // Restore a default ODR (e.g., 100Hz)
    uint8_t odr_reg_val = LIS2DH12_ODR_100HZ | LIS2DH12_XYZ_EN; // 0x57
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, odr_reg_val));

    ESP_LOGI(TAG, "WoM mode disabled");
    return ESP_OK;
}
