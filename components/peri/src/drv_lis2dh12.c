#include "drv_lis2dh12.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

// Register map
#define LIS2DH12_REG_WHO_AM_I 0x0F
#define LIS2DH12_REG_CTRL_REG1 0x20
#define LIS2DH12_REG_CTRL_REG2 0x21
#define LIS2DH12_REG_CTRL_REG3 0x22
#define LIS2DH12_REG_CTRL_REG4 0x23
#define LIS2DH12_REG_CTRL_REG5 0x24
#define LIS2DH12_REG_CTRL_REG6 0x25
#define LIS2DH12_REG_REFERENCE 0x26 // Reading this register resets the HPF
#define LIS2DH12_REG_OUT_X_L 0x28
#define LIS2DH12_REG_INT1_CFG 0x30
#define LIS2DH12_REG_INT1_SRC 0x31
#define LIS2DH12_REG_INT1_THS 0x32
#define LIS2DH12_REG_INT1_DURATION 0x33
#define LIS2DH12_REG_INT2_CFG 0x34
#define LIS2DH12_REG_INT2_SRC 0x35
#define LIS2DH12_REG_INT2_THS 0x36
#define LIS2DH12_REG_INT2_DURATION 0x37
#define LIS2DH12_REG_FIFO_CTRL 0x2E
#define LIS2DH12_REG_FIFO_SRC 0x2F

#define LIS2DH12_WHO_AM_I_VAL 0x33
#define LIS2DH12_SLEEP_TIME 20

// SPI configuration
#define SPI_HOST SPI3_HOST

// WoM operating parameters
// INT_THS register is 7-bit (bit[6:0]), threshold step depends on FS:
//   16mg @ ±2g, 32mg @ ±4g, 62mg @ ±8g, 186mg @ ±16g (datasheet Table 59).
// Max register value 127 → max threshold ≈ 127 * step_mg.
#define WOM_THS_MAX 127u // INT_THS bit[6:0] maximum
#define WOM_THS_MIN 1u   // minimum (1 LSb = step_mg, depends on FS)
#define WOM_ODR_HZ 50.0f // 50 Hz → 1 LSB duration = 20 ms
// CTRL_REG1 value for WoM: ODR=50Hz (0x4), LP=0, XYZ enabled
#define WOM_CTRL_REG1 0x47u
// CTRL_REG4 value for WoM: BDU=1, FS selectable, HR=0 (normal mode)
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
// #define WOM_CTRL_REG2 0x03u
#define WOM_CTRL_REG2 0xC3u

// --- Driver state ---
static spi_device_handle_t s_spi_handle = NULL;
static SemaphoreHandle_t s_spi_mutex = NULL;
static bool g_lis2dh12_initialized = false;
static float s_current_odr = 0.0f;
static lis2dh12_fs_t s_current_fs = LIS2DH12_FS_2G;

static uint16_t lis2dh12_int_ths_step_mg(lis2dh12_fs_t fs)
{
    switch (fs)
    {
    case LIS2DH12_FS_2G:
        return 16u;
    case LIS2DH12_FS_4G:
        return 32u;
    case LIS2DH12_FS_8G:
        return 62u;
    case LIS2DH12_FS_16G:
    default:
        return 186u;
    }
}

static uint8_t lis2dh12_fs_g(lis2dh12_fs_t fs)
{
    switch (fs)
    {
    case LIS2DH12_FS_2G:
        return 2u;
    case LIS2DH12_FS_4G:
        return 4u;
    case LIS2DH12_FS_8G:
        return 8u;
    case LIS2DH12_FS_16G:
    default:
        return 16u;
    }
}

// --- Private Functions (Forward Declaration) ---
static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data);
static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data);
static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len);

// --- Private Functions ---

static esp_err_t lis2dh12_write_reg(uint8_t reg, uint8_t data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {
        .length = 8,
        .cmd = reg & 0x7F, // MSB=0 for write
        .tx_buffer = &data,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_reg(uint8_t reg, uint8_t *data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {
        .length = 8,
        .cmd = reg | 0x80, // MSB=1 for read
        .rx_buffer = data,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t lis2dh12_read_multiple(uint8_t reg, uint8_t *buffer, size_t len)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {
        .length = len * 8,
        .cmd = reg | 0x80 | 0x40, // MSB=1 read, bit6=1 auto-increment
        .rx_buffer = buffer,
    };
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

// --- ODR mapping ---
typedef struct
{
    float odr_hz;
    uint8_t reg_value;
} lis2dh12_odr_map_t;

static const lis2dh12_odr_map_t s_odr_table[] = {
    {1.0f, 0x01},
    {10.0f, 0x02},
    {25.0f, 0x03},
    {50.0f, 0x04},
    {100.0f, 0x05},
    {200.0f, 0x06},
    {400.0f, 0x07},
    {1344.0f, 0x09},
};
static const int ODR_TABLE_SIZE = sizeof(s_odr_table) / sizeof(s_odr_table[0]);

static float lis2dh12_config_odr_callback(float ideal_odr)
{
    // Default to highest available ODR
    float selected_odr = s_odr_table[ODR_TABLE_SIZE - 1].odr_hz;
    uint8_t selected_reg = s_odr_table[ODR_TABLE_SIZE - 1].reg_value;

    for (int i = 0; i < ODR_TABLE_SIZE; i++)
    {
        if (s_odr_table[i].odr_hz >= ideal_odr)
        {
            selected_odr = s_odr_table[i].odr_hz;
            selected_reg = s_odr_table[i].reg_value;
            break;
        }
    }

    s_current_odr = selected_odr;

    uint8_t ctrl_reg1_val = (uint8_t)((selected_reg << 4) | 0x07); // XYZ enabled, normal mode
    if (lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, ctrl_reg1_val) != ESP_OK)
    {
        return 0.0f;
    }
    return selected_odr;
}

// SensorDriver struct for imu_config
SensorDriver_t lis2dh12_driver = {
    .name = "LIS2DH12",
    .config_hardware_odr = lis2dh12_config_odr_callback,
};

// --- Public Functions ---

esp_err_t drv_lis2dh12_init(void)
{
    if (g_lis2dh12_initialized)
    {
        return ESP_OK;
    }

    s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex)
    {
        LOG_ERROR("Failed to create SPI mutex");
        return ESP_ERR_NO_MEM;
    }
    // SPI 总线改由 peri_spi_bus_init 负责初始化，驱动仅添加设备句柄

    // Use canonical pin macros where possible; prefer ICM defines for MISO/MOSI/CLK
    spi_bus_config_t buscfg = {
        .miso_io_num = LIS2DH12_PIN_NUM_SDO,    // 21
        .mosi_io_num = LIS2DH12_PIN_NUM_SDA,    // 35
        .sclk_io_num = LIS2DH12_PIN_NUM_SCL,    // 36
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        LOG_ERRORF("drv_lis2dh12_init: spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 调试阶段：降速至 1MHz 与 LIS2DH12 保持一致
        .mode = 3,                         // 核心修改：改为 Mode 3，与 LIS2DH12 统一，减少时钟线跳变
        .spics_io_num = LIS2DH12_PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 8,
    };
    // Note: spi device expects the bus to be initialized prior to this call.
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("SPI bus add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Soft-reset the device to guarantee a known state on startup.
    // This is crucial after a re-flash, which can leave the sensor
    // in an undefined state that is only cleared by a full power cycle.
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x80);
    if (ret != ESP_OK)
    {
        LOG_ERROR("Failed to soft-reset LIS2DH12");
        return ret;
    }
    // Datasheet: boot takes <5ms. Add generous delay.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Ensure CS is high (inactive) before any transaction
    // This is critical if the bootloader left it in an undefined state
    gpio_set_level(LIS2DH12_PIN_NUM_CS, 1);

    // Give the sensor a moment to stabilize after CS pin config
    // Increase from 20ms to 50ms to ensure sensor is ready
    vTaskDelay(pdMS_TO_TICKS(50));

    // Perform a dummy read to wake up SPI interface (ST sensors sometimes need this)
    uint8_t dummy;
    lis2dh12_read_reg(LIS2DH12_REG_WHO_AM_I, &dummy);

    // Verify device identity with retry
    uint8_t who_am_i = 0;
    for (int i = 0; i < 10; i++)
    {
        ret = lis2dh12_read_reg(LIS2DH12_REG_WHO_AM_I, &who_am_i);
        if (ret == ESP_OK && who_am_i == LIS2DH12_WHO_AM_I_VAL)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(LIS2DH12_SLEEP_TIME));
    }

    LOG_DEBUGF("WHO_AM_I: 0x%02x", who_am_i);
    if (who_am_i != LIS2DH12_WHO_AM_I_VAL)
    {
        LOG_ERRORF("WHO_AM_I check failed. Got 0x%02x, expected 0x%02x", who_am_i, LIS2DH12_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    // Power-on default: BDU=1, FS=±2g, normal mode (HR=0)
    // FS will be overridden by enable_wom() before any measurement is made
    uint8_t ctrl_reg4_val = (uint8_t)((s_current_fs << 4) | 0x80u);
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
    if (ret != ESP_OK)
    {
        LOG_ERROR("Failed to set CTRL_REG4");
        return ret;
    }

    LOG_DEBUG("LIS2DH12 initialized successfully");
    g_lis2dh12_initialized = true;
    return ESP_OK;
}

esp_err_t drv_lis2dh12_set_config(lis2dh12_fs_t fs, float odr)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;

    lis2dh12_config_odr_callback(odr);

    s_current_fs = fs;
    uint8_t ctrl_reg4_val = 0;
    lis2dh12_read_reg(LIS2DH12_REG_CTRL_REG4, &ctrl_reg4_val);
    ctrl_reg4_val = (uint8_t)((ctrl_reg4_val & 0xCFu) | (fs << 4));
    return lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4_val);
}

esp_err_t drv_lis2dh12_get_raw_data(lis2dh12_raw_data_t *data)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!data)
        return ESP_ERR_INVALID_ARG;

    uint8_t buffer[6];
    esp_err_t ret = lis2dh12_read_multiple(LIS2DH12_REG_OUT_X_L, buffer, 6);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // LIS2DH12 output registers are left-justified two's complement.
    // Normal mode (10-bit): right-shift 6 to get the signed 10-bit value.
    // High-resolution (12-bit): >>4. Low-power (8-bit): >>8.
    // WoM config uses normal mode (HR=0, LPen=0) → >>6.
    data->header = 0xAA;
    data->x = (int16_t)((buffer[1] << 8) | buffer[0]) >> 6;
    data->y = (int16_t)((buffer[3] << 8) | buffer[2]) >> 6;
    data->z = (int16_t)((buffer[5] << 8) | buffer[4]) >> 6;
    data->temp = 0;

    return ESP_OK;
}

lis2dh12_fs_t drv_lis2dh12_get_current_fs(void)
{
    return s_current_fs;
}

esp_err_t drv_lis2dh12_self_test(void)
{
    LOG_DEBUG("drv_lis2dh12_self_test is not implemented");
    return ESP_OK;
}

esp_err_t drv_lis2dh12_read_int1_source(uint8_t *source)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!source)
        return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT1_SRC, source);
}

esp_err_t drv_lis2dh12_read_int2_source(uint8_t *source)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!source)
        return ESP_ERR_INVALID_ARG;
    return lis2dh12_read_reg(LIS2DH12_REG_INT2_SRC, source);
}

esp_err_t drv_lis2dh12_read_register(uint8_t reg, uint8_t *value)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!value)
        return ESP_ERR_INVALID_ARG;
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
// CTRL_REG3 bit6 = I1_IA1 → INT1 pin. CTRL_REG6 bit6 = I2_IA2 → INT2 pin.
// ---------------------------------------------------------------------------
esp_err_t drv_lis2dh12_enable_wom(const lis2dh12_wom_cfg_t *wom_cfg)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!wom_cfg)
        return ESP_ERR_INVALID_ARG;

    LOG_DEBUG("Enabling WoM mode (FS=±16g, ODR=50Hz, HPF on both INT1+INT2)...");

    // Step 1 — Power down and clear all control/interrupt registers.
    esp_err_t ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x00);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(LIS2DH12_SLEEP_TIME));

    // Step 2 — Set FS (from config), BDU=1, HR=0 (normal mode).
    lis2dh12_fs_t wom_fs = wom_cfg->fs;
    if (wom_fs > LIS2DH12_FS_16G)
        wom_fs = LIS2DH12_FS_16G;
    s_current_fs = wom_fs;
    uint8_t ctrl_reg4 = (uint8_t)((wom_fs << 4) | 0x80u);
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG4, ctrl_reg4);
    if (ret != ESP_OK)
        return ret;

    // Step 3 — Enable HPF for both INT1 and INT2 (HPM=11, HP_IA1=1, HP_IA2=1).
    //   CTRL_REG2 = 0xC3
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, WOM_CTRL_REG2);
    if (ret != ESP_OK)
        return ret;

    // Step 4 — Enable ODR=50Hz, normal mode, XYZ axes. Wait for stabilisation.
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, WOM_CTRL_REG1);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(LIS2DH12_SLEEP_TIME));

    // Step 5 — Compute and write interrupt thresholds.
    //   INT_THS is 7-bit unsigned, step depends on FS; clamp to [WOM_THS_MIN, WOM_THS_MAX].
    const uint16_t ths_step_mg = lis2dh12_int_ths_step_mg(wom_fs);
    uint8_t thr1 = (uint8_t)(wom_cfg->threshold_mg_int1 / ths_step_mg);
    uint8_t thr2 = (uint8_t)(wom_cfg->threshold_mg_int2 / ths_step_mg);
    if (thr1 > WOM_THS_MAX)
        thr1 = WOM_THS_MAX;
    if (thr2 > WOM_THS_MAX)
        thr2 = WOM_THS_MAX;
    if (thr1 < WOM_THS_MIN)
        thr1 = WOM_THS_MIN;
    if (thr2 < WOM_THS_MIN)
        thr2 = WOM_THS_MIN;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT1_THS, thr1);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT2_THS, thr2);
    if (ret != ESP_OK)
        return ret;
    LOG_DEBUGF("THS1=%u (~%u mg req=%u), THS2=%u (~%u mg req=%u) [FS=±%ug, %umg/LSB]",
               thr1, thr1 * ths_step_mg, wom_cfg->threshold_mg_int1,
               thr2, thr2 * ths_step_mg, wom_cfg->threshold_mg_int2,
               lis2dh12_fs_g(wom_fs), ths_step_mg);

    // Step 6 — Write duration registers (1 LSB = 20 ms at 50 Hz).
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT1_DURATION, wom_cfg->duration_int1);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT2_DURATION, wom_cfg->duration_int2);
    if (ret != ESP_OK)
        return ret;
    LOG_DEBUGF("Duration INT1=%u ms, INT2=%u ms",
               wom_cfg->duration_int1 * 20u, wom_cfg->duration_int2 * 20u);

    // Step 7 — Configure interrupt event logic.
    //
    // IMPORTANT:
    //   Do NOT enable the "low event" bits here. On LIS2DH12, "low event" means
    //   accel lower than the threshold (not necessarily negative). With a large
    //   threshold (e.g. 6000 mg) the condition "accel < THS" can be true most of
    //   the time on all axes at rest, causing continuous interrupts.
    //
    // INT1/INT2: OR combination, high events on X/Y/Z only → 0x2A
    // 传感器只会对正向（Positive）的冲击或倾斜产生反应
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x2A);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x2A);
    if (ret != ESP_OK)
        return ret;

    // Initialise HPF baseline.
    // Reading REFERENCE resets the high-pass filter internal reference.
    // This helps avoid repeated interrupts caused by undefined HPF state after reset.
    {
        uint8_t hpf_ref = 0;
        ret = lis2dh12_read_reg(LIS2DH12_REG_REFERENCE, &hpf_ref);
        if (ret != ESP_OK)
        {
            LOG_ERRORF("Failed to initialize HPF baseline: %s", esp_err_to_name(ret));
            return ret;
        }
        LOG_DEBUGF("HPF baseline initialized (REFERENCE=0x%02X)", hpf_ref);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Step 8 — Latch interrupts on INT1/INT2 (recommended for MCU GPIO + wakeup).
    //   With latching enabled, INT pins stay asserted until INTx_SRC is read.
    //   CTRL_REG5: LIR_INT1=1 (bit3), LIR_INT2=1 (bit1) → 0x0A
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x0A);
    if (ret != ESP_OK)
        return ret;

    // Step 9 — Route interrupts to physical pins.
    //   CTRL_REG3 bit6 = I1_IA1 → IA1 event drives INT1 pin
    //   CTRL_REG6 bit6 = I2_IA2 → IA2 event drives INT2 pin
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x40); // I1_IA1
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x40); // I2_IA2
    if (ret != ESP_OK)
        return ret;

    LOG_DEBUGF("WoM enabled — INT1: >%u mg, INT2: >%u mg",
               thr1 * ths_step_mg, thr2 * ths_step_mg);
    return ESP_OK;
}

esp_err_t drv_lis2dh12_disable_wom(void)
{
    if (!g_lis2dh12_initialized)
        return ESP_ERR_INVALID_STATE;

    LOG_DEBUG("Disabling WoM mode...");
    // Power down first to silence the interrupt generators cleanly
    esp_err_t ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x00);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(LIS2DH12_SLEEP_TIME));

    // Clear all interrupt routing and event configuration
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG2, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG3, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG5, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG6, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT1_CFG, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = lis2dh12_write_reg(LIS2DH12_REG_INT2_CFG, 0x00);
    if (ret != ESP_OK)
        return ret;
    // Restore default operating mode: 100 Hz, normal mode, XYZ enabled
    ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL_REG1, 0x57);
    if (ret != ESP_OK)
        return ret;

    LOG_DEBUG("WoM mode disabled");
    return ESP_OK;
}
