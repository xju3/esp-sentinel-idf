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
#define LIS2DH12_REG_INT1_SRC        0x31
#define LIS2DH12_REG_INT2_CFG        0x34
#define LIS2DH12_REG_INT2_THS        0x36
#define LIS2DH12_REG_INT2_DURATION   0x37
#define LIS2DH12_REG_INT2_SRC        0x35
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
                 LIS2DH12_PIN_NUM_SCL, LIS2DH12_PIN_NUM_SDO, LIS2DH12_PIN_NUM_SDA, LIS2DH12_PIN_NUM_CS);
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

esp_err_t drv_lis2dh12_self_test(void) {
    // STUB
    ESP_LOGW(TAG, "drv_lis2dh12_self_test is not implemented");
    return ESP_OK;
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

// debug helper implementation
esp_err_t drv_lis2dh12_read_register(uint8_t reg, uint8_t *value) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!value) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(reg, value);
}


// drv_lis2dh12.c - enable_wom() 修改

esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg) {

    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!wom_cfg) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Enabling WoM mode...");

    // 1. 先掉电复位所有控制寄存器，确保干净状态
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. 强制设置FS=±16g，BDU=1，HR=0（normal mode）
    //    原因：WoM场景下需要大量程避免饱和；threshold用mg计算时需匹配
    s_current_fs = LIS2DH12_FS_16G;
    uint8_t ctrl_reg4_val = (LIS2DH12_FS_16G << 4) | 0x80; // BDU=1
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val));

    // 3. 上电，50Hz normal mode，XYZ使能
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x47));
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待ODR稳定（至少2个周期=40ms，给100ms余量）

    // 4. 使能HPF：HPM=00（normal mode，自动复位），HPIS1=1，HPIS2=1
    //    注意：HPM=00 不是 10，00才是"normal mode with reset"
    //    0b00001100 = 0x0C
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x0C));

    // 5. 读REFERENCE寄存器一次，强制HPF基线复位到当前DC值
    uint8_t reference_dummy;
    lis2dh12_read_reg(0x26, &reference_dummy); // REG_REFERENCE = 0x26
    vTaskDelay(pdMS_TO_TICKS(200)); // 等待HPF收敛（10个周期@50Hz=200ms）

    // 6. flush采样，确认HPF已收敛（Z轴HPF输出应接近0）
    lis2dh12_raw_data_t baseline_sample;
    for (int i = 0; i < 10; i++) {
        drv_lis2dh12_get_raw_data(&baseline_sample);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    ESP_LOGI(TAG, "HPF baseline after flush: X=%d, Y=%d, Z=%d (should be near 0)",
             baseline_sample.x, baseline_sample.y, baseline_sample.z);

    // 7. 检查基线是否合理（FS=±16g，12mg/LSB，静止时HPF输出应<50 LSB）
    int16_t max_baseline = 50;
    if (abs(baseline_sample.x) > max_baseline ||
        abs(baseline_sample.y) > max_baseline ||
        abs(baseline_sample.z) > max_baseline) {
        ESP_LOGW(TAG, "HPF baseline not settled (X=%d Y=%d Z=%d), consider increasing flush delay",
                 baseline_sample.x, baseline_sample.y, baseline_sample.z);
    }

    // 8. 计算阈值寄存器值（FS=±16g，12mg/LSB，寄存器7位最大127）
    //    INT_THS寄存器只有bit[6:0]有效，最大值127
    const uint16_t sensitivity_mg_lsb = 12; // FS=±16g
    uint8_t thr1 = (uint8_t)(wom_cfg->threshold_mg_int1 / sensitivity_mg_lsb);
    uint8_t thr2 = (uint8_t)(wom_cfg->threshold_mg_int2 / sensitivity_mg_lsb);
    // 钳位到7位最大值
    if (thr1 > 127) thr1 = 127;
    if (thr2 > 127) thr2 = 127;
    // 防止阈值过低（<5 LSB）导致噪声触发
    if (thr1 < 5) thr1 = 5;
    if (thr2 < 5) thr2 = 5;

    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_THS, thr1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_THS, thr2));
    ESP_LOGI(TAG, "THS1=%d (~%dmg), THS2=%d (~%dmg) @ FS=±16g",
             thr1, thr1 * sensitivity_mg_lsb,
             thr2, thr2 * sensitivity_mg_lsb);

    // 9. Duration（50Hz，1LSB=20ms）
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_DURATION, wom_cfg->duration_int1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_DURATION, wom_cfg->duration_int2));

    // 10. 中断路由
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x40)); // INT1引脚←IA1
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x20)); // INT2引脚←IA2
    // 注意CTRL_REG6 bit5=I2_IA2，不是bit6

    // 11. 中断事件配置（OR模式，XYZ高事件）
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x2A));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x2A));

    // 12. 无锁存，脉冲模式
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00));

    ESP_LOGI(TAG, "WoM enabled. INT1: shock>%dmg, INT2: posture>%dmg",
             wom_cfg->threshold_mg_int1, wom_cfg->threshold_mg_int2);
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
