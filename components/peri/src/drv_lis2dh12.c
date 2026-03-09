#include "drv_lis2dh12.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>


// Register map
#define LIS2DH12_REG_WHO_AM_I        0x0F
#define LIS2DH12_REG_CTRL_REG1       0x20
#define LIS2DH12_REG_CTRL_REG2       0x21
#define LIS2DH12_REG_CTRL_REG3       0x22
#define LIS2DH12_REG_CTRL_REG4       0x23
#define LIS2DH12_REG_CTRL_REG5       0x24
#define LIS2DH12_REG_CTRL_REG6       0x25
#define LIS2DH12_REG_REFERENCE       0x26   // Reading this register resets the HPF
#define LIS2DH12_REG_OUT_X_L         0x28
#define LIS2DH12_REG_INT1_CFG        0x30
#define LIS2DH12_REG_INT1_SRC        0x31
#define LIS2DH12_REG_INT1_THS        0x32
#define LIS2DH12_REG_INT1_DURATION   0x33
#define LIS2DH12_REG_INT2_CFG        0x34
#define LIS2DH12_REG_INT2_SRC        0x35
#define LIS2DH12_REG_INT2_THS        0x36
#define LIS2DH12_REG_INT2_DURATION   0x37
#define LIS2DH12_REG_FIFO_CTRL       0x2E
#define LIS2DH12_REG_FIFO_SRC        0x2F

#define LIS2DH12_WHO_AM_I_VAL        0x33

// SPI configuration
#define SPI_HOST SPI2_HOST

// WoM operating parameters
// FS=±16g, normal mode (10-bit)
// Sensitivity per datasheet Table 59/68: 1 LSb = 186 mg @ FS=±16g, normal mode
// (Note: 48 mg/LSb is the HIGH-RESOLUTION mode value — do not use that here)
// INT_THS register is 7-bit (bit[6:0]), max value = 127 → 127 × 186 = 23622 mg
#define WOM_FS              LIS2DH12_FS_16G
#define WOM_SENSITIVITY_MG  186u    // mg per LSB, normal mode, FS=±16g (datasheet Table 59)
#define WOM_THS_MAX         127u    // INT_THS bit[6:0] maximum
#define WOM_THS_MIN         1u      // minimum (1 LSb = 186 mg @ 186mg/LSB)
#define WOM_ODR_HZ          50.0f   // 50 Hz → 1 LSB duration = 20 ms
// CTRL_REG1 value for WoM: ODR=50Hz (0x4), LP=0, XYZ enabled
#define WOM_CTRL_REG1       0x47u
// CTRL_REG4 value for WoM: BDU=1, FS=±16g (0x3<<4), HR=0 (normal mode)
#define WOM_CTRL_REG4       ((uint8_t)((LIS2DH12_FS_16G << 4) | 0x80u))
// CTRL_REG2 value:
//   CTRL_REG2: [HPM1|HPM0|HPCF2|HPCF1|FDS|HPCLICK|HP_IA2|HP_IA1]
//               bit7  bit6  bit5  bit4  bit3  bit2   bit1   bit0
//
//   Both INT1 and INT2 use HPF so gravity is suppressed on both channels.
//
//   INT1 (vibration/shock): HPM=11 (autoreset on interrupt) → HP_IA1=1 (bit0)
//     After each INT1 event the HPF baseline auto-resets to current acceleration.
//     Dynamic "delta from current state" detector — ideal for impulse detection.
//
//   INT2 (posture/tilt): HPM=00 (normal mode) → HP_IA2=1 (bit1)
//     Baseline is fixed at the moment REFERENCE register is read (startup).
//     When the device tilts/flips, gravity shifts on one or more axes; the HPF
//     output deviates from zero and stays elevated until the filter reconverges
//     (~seconds). Combined with duration=10 (200 ms) this reliably detects
//     sustained orientation changes while rejecting short bumps.
//
//   HPM field is shared — HPM=11 takes priority and applies to both channels
//   when both HP_IA1 and HP_IA2 are set. To get different behaviour per channel
//   we use the same HPM=11 for both, but give INT2 a much longer duration so
//   only sustained deviations (tilts) trigger it while transient shocks (which
//   reset the baseline immediately) do not accumulate enough duration counts.
//
//   Both INT1 and INT2 use HPF for shock/vibration detection.
//   HPM=11 (autoreset on interrupt): baseline resets after each event on
//   either channel, keeping both detectors sensitive at all times.
//   HP_IA1=1 (bit0), HP_IA2=1 (bit1).
//
//   HPM=11 → bit7=1, bit6=1; HP_IA2=1 → bit1=1; HP_IA1=1 → bit0=1
//   CTRL_REG2 = 0b11000011 = 0xC3
#define WOM_CTRL_REG2           0xC3u

// --- Driver state ---
static spi_device_handle_t s_spi_handle = NULL;
static SemaphoreHandle_t   s_spi_mutex  = NULL;
static bool                s_initialized  = false;
static float               s_current_odr  = 0.0f;
static lis2dh12_fs_t       s_current_fs   = LIS2DH12_FS_2G;

// --- Private Functions (Forward Declaration) ---
static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data);
static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data);
static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len);

// --- Private Functions ---

static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length    = 8,
        .addr      = reg & 0x7F,   // MSB=0 for write
        .tx_buffer = &data,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length    = 8,
        .addr      = reg | 0x80,   // MSB=1 for read
        .rx_buffer = data,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len) {
    if (!s_spi_mutex) return ESP_FAIL;
    spi_transaction_t t = {
        .length    = len * 8,
        .addr      = reg | 0x80 | 0x40,  // MSB=1 read, bit6=1 auto-increment
        .rx_buffer = buffer,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

// --- ODR mapping ---
typedef struct {
    float   odr_hz;
    uint8_t reg_value;
} lis2dh12_odr_map_t;

static const lis2dh12_odr_map_t s_odr_table[] = {
    {1.0f,    0x01}, {10.0f,   0x02}, {25.0f,   0x03}, {50.0f,   0x04},
    {100.0f,  0x05}, {200.0f,  0x06}, {400.0f,  0x07}, {1344.0f, 0x09},
};
static const int ODR_TABLE_SIZE = sizeof(s_odr_table) / sizeof(s_odr_table[0]);

static float lis2dh12_config_odr_callback(float ideal_odr) {
    // Default to highest available ODR
    float   selected_odr = s_odr_table[ODR_TABLE_SIZE - 1].odr_hz;
    uint8_t selected_reg = s_odr_table[ODR_TABLE_SIZE - 1].reg_value;

    for (int i = 0; i < ODR_TABLE_SIZE; i++) {
        if (s_odr_table[i].odr_hz >= ideal_odr) {
            selected_odr = s_odr_table[i].odr_hz;
            selected_reg = s_odr_table[i].reg_value;
            break;
        }
    }

    s_current_odr = selected_odr;

    uint8_t ctrl_reg1_val = (uint8_t)((selected_reg << 4) | 0x07); // XYZ enabled, normal mode
    if (lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, ctrl_reg1_val) != ESP_OK) {
        return 0.0f;
    }
    return selected_odr;
}

// SensorDriver struct for imu_config
SensorDriver_t lis2dh12_driver = {
    .name               = "LIS2DH12",
    .config_hardware_odr = lis2dh12_config_odr_callback,
};

// --- Public Functions ---

esp_err_t drv_lis2dh12_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }

    s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex) {
        LOG_ERROR("Failed to create SPI mutex");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num    = LIS2DH12_PIN_NUM_SDA,
        .mosi_io_num    = LIS2DH12_PIN_NUM_SDO,
        .sclk_io_num    = LIS2DH12_PIN_NUM_SCL,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz = 4092, // 关键：即使只传小数据，也要为共享总线的大数据设备(ICM)预留空间
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        LOG_ERRORF("SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz
        .mode           = 0,
        .spics_io_num   = LIS2DH12_PIN_NUM_CS,
        .queue_size     = 7,
        .address_bits   = 8,
    };
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK) {
        LOG_ERRORF("SPI bus add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify device identity
    uint8_t who_am_i = 0;
    ret = lis2dh12_read_reg(LIS2DH12_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK || who_am_i != LIS2DH12_WHO_AM_I_VAL) {
        LOG_ERRORF("WHO_AM_I check failed. Got 0x%02x, expected 0x%02x", who_am_i, LIS2DH12_WHO_AM_I_VAL);
        LOG_ERROR("Please verify:");
        LOG_ERRORF("  1. Hardware connections (SCL=%d, MOSI=%d, MISO=%d, CS=%d)",
                 LIS2DH12_PIN_NUM_SCL, LIS2DH12_PIN_NUM_SDO, LIS2DH12_PIN_NUM_SDA, LIS2DH12_PIN_NUM_CS);
        LOG_ERROR("  2. LIS2DH12 power supply");
        LOG_ERROR("  3. SPI clock frequency (currently 1 MHz)");
        return ESP_ERR_NOT_FOUND;
    }

    // Power-on default: BDU=1, FS=±2g, normal mode (HR=0)
    // FS will be overridden by enable_wom() before any measurement is made
    uint8_t ctrl_reg4_val = (uint8_t)((s_current_fs << 4) | 0x80u);
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
    if (ret != ESP_OK) {
        LOG_ERROR("Failed to set CTRL_REG4");
        return ret;
    }

    LOG_DEBUG("LIS2DH12 initialized successfully");
    s_initialized = true;
    return ESP_OK;
}

esp_err_t drv_lis2dh12_set_config(lis2dh12_fs_t fs, float odr) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    lis2dh12_config_odr_callback(odr);

    s_current_fs = fs;
    uint8_t ctrl_reg4_val = 0;
    lis2dh12_read_reg(LIS2DH12_REG_CTRL_REG4, &ctrl_reg4_val);
    ctrl_reg4_val = (uint8_t)((ctrl_reg4_val & 0xCFu) | (fs << 4));
    return lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
}

esp_err_t drv_lis2dh12_get_raw_data(lis2dh12_raw_data_t *data) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!data) return ESP_ERR_INVALID_ARG;

    uint8_t buffer[6];
    esp_err_t ret = lis2dh12_read_multiple(LIS2DH12_REG_OUT_X_L, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // LIS2DH12 output registers are left-justified two's complement.
    // Normal mode (10-bit): right-shift 6 to get the signed 10-bit value.
    // High-resolution (12-bit): >>4. Low-power (8-bit): >>8.
    // WoM config uses normal mode (HR=0, LPen=0) → >>6.
    data->header = 0xAA;
    data->x      = (int16_t)((buffer[1] << 8) | buffer[0]) >> 6;
    data->y      = (int16_t)((buffer[3] << 8) | buffer[2]) >> 6;
    data->z      = (int16_t)((buffer[5] << 8) | buffer[4]) >> 6;
    data->temp   = 0;

    return ESP_OK;
}

lis2dh12_fs_t drv_lis2dh12_get_current_fs(void) {
    return s_current_fs;
}

esp_err_t drv_lis2dh12_self_test(void) {
    LOG_DEBUG("drv_lis2dh12_self_test is not implemented");
    return ESP_OK;
}

esp_err_t drv_lis2dh12_read_int1_source(uint8_t *source) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!source) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT1_SRC, source);
}

esp_err_t drv_lis2dh12_read_int2_source(uint8_t *source) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!source) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT2_SRC, source);
}

esp_err_t drv_lis2dh12_read_register(uint8_t reg, uint8_t *value) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!value) return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(reg, value);
}

// ---------------------------------------------------------------------------
// drv_lis2dh12_enable_wom
//
// Both INT1 and INT2 are shock/vibration detectors with different thresholds.
// HPF active on both (HP_IA1=1, HP_IA2=1), HPM=11 (autoreset on interrupt).
// The HPF removes the static gravity component so only dynamic acceleration
// changes trigger the comparators. After each event the baseline auto-resets.
//
// INT1 — primary vibration / shock warning
//   Lower threshold → catches routine vibration and moderate shocks.
//   Program uses INT1 as a signal to begin further health checks.
//   INT1_CFG=0x2A: AOI=0 (OR), high events on X/Y/Z.
//
// INT2 — severe shock / critical event
//   Higher threshold → only fires on serious impacts.
//   INT2 firing means a more severe problem has occurred.
//   INT2_CFG=0x2A: same logic as INT1, different threshold register.
//
// FS=±16g, normal mode → 186 mg/LSB. ODR=50Hz → 1 duration LSB = 20 ms.
// CTRL_REG3 bit6 = I1_IA1 → INT1 pin. CTRL_REG6 bit5 = I2_IA2 → INT2 pin.
// ---------------------------------------------------------------------------
esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!wom_cfg)        return ESP_ERR_INVALID_ARG;

    LOG_DEBUG("Enabling WoM mode (FS=±16g, ODR=50Hz, HPF on both INT1+INT2)...");

    // Step 1 — Power down and clear all control/interrupt registers.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG,  0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG,  0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2 — Set FS=±16g, BDU=1, HR=0 (normal mode → 186 mg/LSB).
    s_current_fs = WOM_FS;
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, WOM_CTRL_REG4));

    // Step 3 — Enable HPF for both INT1 and INT2 (HPM=11, HP_IA1=1, HP_IA2=1).
    //   CTRL_REG2 = 0xC3
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, WOM_CTRL_REG2));

    // Step 4 — Enable ODR=50Hz, normal mode, XYZ axes. Wait for stabilisation.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, WOM_CTRL_REG1));
    vTaskDelay(pdMS_TO_TICKS(50));

    // Step 5 — Compute and write interrupt thresholds.
    //   INT_THS is 7-bit unsigned, 1 LSb = 186 mg @ FS=±16g normal mode.
    //   Clamp to [WOM_THS_MIN, WOM_THS_MAX].
    uint8_t thr1 = (uint8_t)(wom_cfg->threshold_mg_int1 / WOM_SENSITIVITY_MG);
    uint8_t thr2 = (uint8_t)(wom_cfg->threshold_mg_int2 / WOM_SENSITIVITY_MG);
    if (thr1 > WOM_THS_MAX) thr1 = WOM_THS_MAX;
    if (thr2 > WOM_THS_MAX) thr2 = WOM_THS_MAX;
    if (thr1 < WOM_THS_MIN) thr1 = WOM_THS_MIN;
    if (thr2 < WOM_THS_MIN) thr2 = WOM_THS_MIN;
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_THS, thr1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_THS, thr2));
    LOG_DEBUGF("THS1=%u (~%u mg), THS2=%u (~%u mg) [FS=±16g, 186mg/LSB]",
             thr1, thr1 * WOM_SENSITIVITY_MG,
             thr2, thr2 * WOM_SENSITIVITY_MG);

    // Step 6 — Write duration registers (1 LSB = 20 ms at 50 Hz).
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_DURATION, wom_cfg->duration_int1));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_DURATION, wom_cfg->duration_int2));
    LOG_DEBUGF("Duration INT1=%u ms, INT2=%u ms",
             wom_cfg->duration_int1 * 20u, wom_cfg->duration_int2 * 20u);

    // Step 7 — Configure interrupt event logic.
    //
    //   INT1 (vibration/shock) — HPF is active, so output is AC-coupled.
    //     Only high events needed: a shock pushes HPF output positive on impact.
    //     AOI=0 (OR), XHIE=1, YHIE=1, ZHIE=1 → 0x2A
    //
    //   INT1 and INT2 both use OR, high events on X/Y/Z → 0x2A
    //   Different behaviour comes from different threshold registers only.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x2A));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x2A));

    // Step 8 — Pulse mode (no latch). The interrupt pin pulses for 1/ODR = 20 ms
    //   then returns low automatically. The ISR re-enables the GPIO after handling.
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00));

    // Step 9 — Route interrupts to physical pins.
    //   CTRL_REG3 bit6 = I1_IA1 → IA1 event drives INT1 pin
    //   CTRL_REG6 bit5 = I2_IA2 → IA2 event drives INT2 pin
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x40)); // I1_IA1
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x20)); // I2_IA2

    LOG_DEBUGF("WoM enabled — INT1: >%u mg, INT2: >%u mg",
             thr1 * WOM_SENSITIVITY_MG, thr2 * WOM_SENSITIVITY_MG);
    return ESP_OK;
}

esp_err_t drv_lis2dh12_disable_wom(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    LOG_DEBUG("Disabling WoM mode...");

    // Power down first to silence the interrupt generators cleanly
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(5));

    // Clear all interrupt routing and event configuration
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG,  0x00));
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG,  0x00));

    // Restore default operating mode: 100 Hz, normal mode, XYZ enabled
    ESP_ERROR_CHECK(lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x57));

    LOG_DEBUG("WoM mode disabled");
    return ESP_OK;
}