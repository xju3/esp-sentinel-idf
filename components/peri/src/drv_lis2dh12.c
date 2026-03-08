#include "drv_lis2dh12.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "LIS2DH12";

// Register map
#define LIS2DH12_REG_WHO_AM_I      0x0F
#define LIS2DH12_REG_CTRL_REG1      0x20
#define LIS2DH12_REG_CTRL_REG2      0x21
#define LIS2DH12_REG_CTRL_REG3      0x22
#define LIS2DH12_REG_CTRL_REG4      0x23
#define LIS2DH12_REG_CTRL_REG5      0x24
#define LIS2DH12_REG_CTRL_REG6      0x25
#define LIS2DH12_REG_OUT_X_L         0x28
#define LIS2DH12_REG_INT1_CFG        0x30
#define LIS2DH12_REG_INT1_THS        0x32
#define LIS2DH12_REG_INT1_DURATION   0x33
#define LIS2DH12_REG_INT2_CFG        0x34
#define LIS2DH12_REG_INT2_THS        0x36
#define LIS2DH12_REG_INT2_DURATION   0x37
#define LIS2DH12_REG_FIFO_CTRL       0x2E
#define LIS2DH12_REG_FIFO_SRC        0x2F

#define LIS2DH12_WHO_AM_I_VAL      0x33

// SPI configuration
#define SPI_HOST SPI2_HOST

// --- Driver state ---
static spi_device_handle_t s_spi_handle = NULL;
static SemaphoreHandle_t s_spi_mutex = NULL;
static bool s_initialized = false;
static float s_current_odr = 0.0f;
static lis2dh12_fs_t s_current_fs = LIS2DH12_FS_2G;

// --- FIFO Capture state ---
static TaskHandle_t s_fifo_reader_task_handle = NULL;
static SemaphoreHandle_t s_fifo_data_ready_sem = NULL;
static lis2dh12_data_cb_t s_fifo_data_cb = NULL;
static uint8_t s_fifo_watermark_level = 0;

// --- Private Functions (Forward Declaration) ---
static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data);
static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data);
static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len);

// --- Private Functions ---

// ISR for FIFO watermark interrupt
static void IRAM_ATTR lis2dh12_fifo_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_fifo_data_ready_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Task to read data from FIFO
static void lis2dh12_fifo_reader_task(void *arg) {
    lis2dh12_raw_data_t *sample_buffer = NULL;
    ESP_LOGI(TAG, "FIFO reader task started.");

    // Allocate buffer once
    sample_buffer = malloc(sizeof(lis2dh12_raw_data_t) * 32); // Max FIFO size
    if (sample_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate FIFO sample buffer!");
        vTaskDelete(NULL);
        return;
    }

    while(1) {
        if(xSemaphoreTake(s_fifo_data_ready_sem, portMAX_DELAY)) {
            uint8_t fifo_src_val = 0;
            lis2dh12_read_reg(LIS2DH12_REG_FIFO_SRC, &fifo_src_val);

            // FSS bits (0-4) hold the number of unread samples
            uint8_t unread_samples = fifo_src_val & 0x1F;

            if (unread_samples > 0) {
                esp_err_t ret = lis2dh12_read_multiple(LIS2DH12_REG_OUT_X_L, (uint8_t*)sample_buffer, unread_samples * 6);
                if (ret == ESP_OK) {
                    if (s_fifo_data_cb) {
                        s_fifo_data_cb(sample_buffer, unread_samples);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to read FIFO data");
                }
            }
        }
    }
    free(sample_buffer);
    vTaskDelete(NULL);
}


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
        .miso_io_num = LIS2DH12_PIN_NUM_SDA,
        .mosi_io_num = LIS2DH12_PIN_NUM_SDO,
        .sclk_io_num = LIS2DH12_PIN_NUM_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000, // 5 MHz
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
    data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return ESP_OK;
}

esp_err_t drv_lis2dh12_capture(uint32_t duration_ms, lis2dh12_data_cb_t cb) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!cb || s_current_odr <= 0.0f) return ESP_ERR_INVALID_ARG;

    uint32_t total_samples = (uint32_t)(s_current_odr * duration_ms / 1000.0f);
    if (total_samples == 0) return ESP_OK;

    TickType_t delay_ticks = pdMS_TO_TICKS(1000.0f / s_current_odr);
    if (delay_ticks == 0) delay_ticks = 1;
    
    lis2dh12_raw_data_t sample;

    for (uint32_t i = 0; i < total_samples; i++) {
        esp_err_t ret = drv_lis2dh12_get_raw_data(&sample);
        if (ret == ESP_OK) {
            cb(&sample, 1);
        }
        vTaskDelay(delay_ticks);
    }
    
    return ESP_OK;
}

esp_err_t drv_lis2dh12_start_fifo_capture(uint8_t watermark_level, lis2dh12_data_cb_t cb) {
    if (!s_initialized || s_fifo_reader_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (watermark_level == 0 || watermark_level > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Starting FIFO capture with watermark %d", watermark_level);

    s_fifo_watermark_level = watermark_level;
    s_fifo_data_cb = cb;

    s_fifo_data_ready_sem = xSemaphoreCreateBinary();
    if (s_fifo_data_ready_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_created = xTaskCreate(lis2dh12_fifo_reader_task, "lis2dh12_fifo_task", 4096, NULL, 12, &s_fifo_reader_task_handle);
    if (task_created != pdPASS) {
        vSemaphoreDelete(s_fifo_data_ready_sem);
        s_fifo_data_ready_sem = NULL;
        return ESP_FAIL;
    }

    // --- Configure Sensor ---
    // 1. Disable WoM interrupts before changing settings
    lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00);

    // 2. Enable FIFO, set Stream mode and watermark level
    lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x40); // FIFO_EN = 1
    uint8_t fifo_ctrl_val = (0b10 << 6) | (watermark_level & 0x1F); // Stream mode, set watermark
    lis2dh12_write_reg(LIS2DH12_REG_FIFO_CTRL, fifo_ctrl_val);
    
    // 3. Route FIFO watermark interrupt to INT1
    lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x04); // I1_WTM = 1

    // --- Configure GPIO ISR ---
    // Remove handler if it was attached by WoM listener
    gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
    // Add our specific handler
    esp_err_t ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT1, lis2dh12_fifo_isr_handler, (void*) LIS2DH12_PIN_NUM_INT1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add FIFO ISR handler");
        drv_lis2dh12_stop_fifo_capture();
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t drv_lis2dh12_stop_fifo_capture(void) {
    if (s_fifo_reader_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Stopping FIFO capture");

    // --- Disable GPIO ISR ---
    gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);

    // --- Disable Sensor FIFO ---
    // Route nothing to INT1
    lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00);
    // Disable FIFO by setting Bypass mode
    lis2dh12_write_reg(LIS2DH12_REG_FIFO_CTRL, 0x00);
    // Clear FIFO_EN bit
    lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00);

    // --- Cleanup RTOS objects ---
    if (s_fifo_reader_task_handle) {
        vTaskDelete(s_fifo_reader_task_handle);
        s_fifo_reader_task_handle = NULL;
    }
    if (s_fifo_data_ready_sem) {
        vSemaphoreDelete(s_fifo_data_ready_sem);
        s_fifo_data_ready_sem = NULL;
    }
    s_fifo_data_cb = NULL;
    s_fifo_watermark_level = 0;

    return ESP_OK;
}


esp_err_t drv_lis2dh12_self_test(void) {
    // STUB
    ESP_LOGW(TAG, "drv_lis2dh12_self_test is not implemented");
    return ESP_OK;
}

esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!wom_cfg) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Enabling WoM mode...");

    // 1. Set ODR to a low-power friendly rate, e.g., 50Hz, and enter normal mode.
    uint8_t odr_reg_val = 0x47; // 50 Hz, normal mode, X/Y/Z enabled
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, odr_reg_val));
    vTaskDelay(pdMS_TO_TICKS(20)); // Wait for settings to apply

    // 2. Enable High-Pass filter for interrupt functions
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x09)); // HPF enabled for INT1

    // 3. Configure INT1 and INT2 routing
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x40)); // I1_IA1 interrupt on INT1 pin
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x40)); // I2_IA2 interrupt on INT2 pin

    // 4. Set Thresholds. The value is dependent on the Full-Scale selection.
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

    // 5. Set Durations. Duration = value / ODR.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_DURATION, wom_cfg->duration_int1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_DURATION, wom_cfg->duration_int2));

    // 6. Configure interrupt event logic.
    // Use OR combination and enable high events on all axes.
    uint8_t int_cfg = 0x2A; // AOI=0 (OR), ZHIE=1, YHIE=1, XHIE=1
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, int_cfg));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, int_cfg));
    
    // 7. Latch interrupts on INT1 and INT2 to read the source register
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x0A)); // LIR_INT1=1, LIR_INT2=1

    ESP_LOGI(TAG, "WoM mode enabled with INT1_THS=%dmg, INT2_THS=%dmg", thr1 * sensitivity_mg_lsb, thr2 * sensitivity_mg_lsb);

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
    uint8_t odr_reg_val = 0x57; // 100 Hz, normal mode, X/Y/Z enabled
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, odr_reg_val));

    ESP_LOGI(TAG, "WoM mode disabled");
    return ESP_OK;
}

