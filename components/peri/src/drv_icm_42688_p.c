#include "drv_icm_42688_p.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "logger.h"
#include "esp_log.h"
#include <string.h>

#define ICM_REG_DEVICE_CONFIG 0x11
#define ICM_REG_FIFO_CONFIG 0x16
#define ICM_REG_ACCEL_CONFIG0 0x50
#define ICM_REG_FIFO_CONFIG1 0x5F
#define ICM_REG_WHO_AM_I 0x75
#define ICM_REG_FIFO_DATA 0x30

#define DMA_CHUNK_SAMPLES 128
#define DMA_CHUNK_BYTES (DMA_CHUNK_SAMPLES * sizeof(imu_raw_data_t))


static SemaphoreHandle_t s_spi_mutex = NULL;
static spi_device_handle_t s_spi_handle = NULL;
static icm_data_cb_t s_data_cb = NULL;

static uint8_t *s_dma_ping = NULL;
static uint8_t *s_dma_pong = NULL;
static bool icm_42688_p_initialized = false;
static volatile bool s_stream_running = false;
static TaskHandle_t s_dma_task_handle = NULL;

/* ========================================================================= *
 * 底层 SPI 读写辅助函数 (加锁)
 * ========================================================================= */
static esp_err_t icm_write_reg(uint8_t reg, uint8_t data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {.length = 8, .cmd = reg, .tx_buffer = &data};
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t icm_read_reg(uint8_t reg, uint8_t *data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {.length = 8, .cmd = reg | 0x80, .rx_buffer = data};
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

/* ========================================================================= *
 * HAL 硬件抽象层 ODR 搓配回调
 * ========================================================================= */
typedef struct
{
    float odr_hz;
    uint8_t reg_value;
} ICM_ODR_Map_t;

static const ICM_ODR_Map_t icm_odr_table[] = {
    {12.5f, 0x0B}, {25.0f, 0x0A}, {50.0f, 0x09}, {100.0f, 0x08}, {200.0f, 0x07}, {500.0f, 0x0F}, {1000.0f, 0x06}, {2000.0f, 0x05}, {4000.0f, 0x04}, {8000.0f, 0x03}, {16000.0f, 0x02}, {32000.0f, 0x01}};

static const int ICM_ODR_TABLE_SIZE = sizeof(icm_odr_table) / sizeof(icm_odr_table[0]);
static float ICM42688_Config_ODR_Callback(float ideal_odr)
{
    float selected_odr = icm_odr_table[ICM_ODR_TABLE_SIZE - 1].odr_hz;
    uint8_t selected_reg = icm_odr_table[ICM_ODR_TABLE_SIZE - 1].reg_value;

    for (int i = 0; i < ICM_ODR_TABLE_SIZE; i++)
    {
        if (icm_odr_table[i].odr_hz >= ideal_odr)
        {
            selected_odr = icm_odr_table[i].odr_hz;
            selected_reg = icm_odr_table[i].reg_value;
            break;
        }
    }

    // 【核心修复】读-改-写：只更新 ODR 位 (Bit 3:0)，保留 FS 位 (Bit 7:5)
    uint8_t current_cfg = 0;
    icm_read_reg(ICM_REG_ACCEL_CONFIG0, &current_cfg);
    current_cfg = (current_cfg & 0xF0) | selected_reg;
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, current_cfg);

    return selected_odr;
}

SensorDriver_t icm42688_driver = {
    .name = "ICM-42688-P",
    .config_hardware_odr = ICM42688_Config_ODR_Callback};

/* ========================================================================= *
 * WoM 守卫分支
 * ========================================================================= */
void drv_icm42688_clear_wom_interrupt(void)
{
    uint8_t dummy = 0;
    icm_read_reg(0x2D, &dummy);
    icm_read_reg(0x37, &dummy);
    icm_read_reg(0x38, &dummy);
}

esp_err_t enable_icm42688p_wom(uint16_t threshold_mg)
{
    icm_write_reg(0x57, 0x00);
    icm_write_reg(0x65, 0x00);
    icm_write_reg(0x66, 0x00);
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x00);
    icm_write_reg(0x4B, 0x02);
    vTaskDelay(pdMS_TO_TICKS(5));

    icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, 0x06);   // 写死 50Hz, 16G 供 WoM 使用
    icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x02); // 开启 LP 模式
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t status = 0xFF;
    for (int i = 0; i < 20; i++)
    {
        icm_read_reg(0x2D, &status);
        icm_read_reg(0x37, &status);
        if (status == 0x00)
            break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (threshold_mg > 1000)
        threshold_mg = 1000;
    uint8_t threshold_val = (uint8_t)(threshold_mg * 2.048f) ?: 1;

    icm_write_reg(0x76, 0x04);
    icm_write_reg(0x4A, threshold_val);
    icm_write_reg(0x4B, threshold_val);
    icm_write_reg(0x4C, threshold_val);
    icm_write_reg(0x76, 0x00);

    drv_icm42688_clear_wom_interrupt();
    icm_write_reg(0x14, 0x07);
    icm_write_reg(0x66, 0x1C);
    icm_write_reg(0x57, 0x05);

    return ESP_OK;
}

esp_err_t disable_icm42688p_wom(void)
{
    icm_write_reg(0x57, 0x00);
    icm_write_reg(0x65, 0x00);
    icm_write_reg(0x66, 0x00);
    icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));

    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x00);
    icm_write_reg(0x4B, 0x02);
    vTaskDelay(pdMS_TO_TICKS(2));

    drv_icm42688_clear_wom_interrupt();
    return ESP_OK;
}

/* ========================================================================= *
 * 基础硬件初始化与配置
 * ========================================================================= */
esp_err_t drv_icm42688_init()
{
    if (icm_42688_p_initialized)
        return ESP_OK;
    if (s_spi_mutex == NULL)
        s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex)
        return ESP_ERR_NO_MEM;

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
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 调试阶段：降速至 1MHz 与 LIS2DH12 保持一致
        .mode = 0,                         // 核心修改：改为 Mode 3，与 LIS2DH12 统一，减少时钟线跳变
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 8,
    };
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK)
        return ret;

    // ESP32-S3 PSRAM DMA 限制：必须按 Cache Line (32字节) 对齐
    // 使用 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT 确保分配在 PSRAM
    s_dma_ping = (uint8_t *)heap_caps_aligned_alloc(32, DMA_CHUNK_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_dma_pong = (uint8_t *)heap_caps_aligned_alloc(32, DMA_CHUNK_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dma_ping || !s_dma_pong)
        return ESP_ERR_NO_MEM;

    icm_write_reg(ICM_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t who_am_i = 0;
    icm_read_reg(ICM_REG_WHO_AM_I, &who_am_i);
    LOG_DEBUGF("WHO_AM_I: 0x%02x", who_am_i);
    if (who_am_i != 0x47)
        return ESP_ERR_NOT_FOUND;

    icm_42688_p_initialized = true;
    return ESP_OK;
}

esp_err_t drv_icm42688_config(const icm_cfg_t *cfg)
{
    if (!cfg || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;

    icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x03); // 仅开 Accel LN 模式
    vTaskDelay(pdMS_TO_TICKS(20));

    // 【核心修复】读-改-写：只更新 FS 位 (Bit 7:5)，绝不破坏 HAL 设好的 ODR
    uint8_t current_cfg = 0;
    icm_read_reg(ICM_REG_ACCEL_CONFIG0, &current_cfg);
    current_cfg = (current_cfg & 0x1F) | (cfg->fs << 5);
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, current_cfg);

    icm_write_reg(ICM_REG_FIFO_CONFIG1, 0x05);
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x40);

    return ESP_OK;
}

/* ========================================================================= *
 * DMA 任务流控
 * ========================================================================= */
static void icm_dma_worker_task(void *arg)
{
    spi_transaction_t trans[2] = {0};
    trans[0].length = DMA_CHUNK_BYTES * 8;
    trans[0].rx_buffer = s_dma_ping;
    trans[0].cmd = ICM_REG_FIFO_DATA | 0x80;
    trans[1].length = DMA_CHUNK_BYTES * 8;
    trans[1].rx_buffer = s_dma_pong;
    trans[1].cmd = ICM_REG_FIFO_DATA | 0x80;

    uint8_t ping_pong_flag = 0;

    while (s_stream_running)
    {
        vTaskDelay(pdMS_TO_TICKS(125));

        spi_transaction_t *t = (ping_pong_flag == 0) ? &trans[0] : &trans[1];
        ping_pong_flag = !ping_pong_flag;

        spi_device_queue_trans(s_spi_handle, t, portMAX_DELAY);

        spi_transaction_t *p_trans;
        if (spi_device_get_trans_result(s_spi_handle, &p_trans, portMAX_DELAY) == ESP_OK)
        {
            if (s_stream_running && s_data_cb)
            {
                s_data_cb((const imu_raw_data_t *)p_trans->rx_buffer, DMA_CHUNK_SAMPLES);
            }
        }
    }

    spi_transaction_t *p_trans;
    while (spi_device_get_trans_result(s_spi_handle, &p_trans, 0) == ESP_OK)
    {
    }
    s_dma_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t drv_icm42688_start_stream(icm_data_cb_t cb)
{
    if (!cb || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;
    if (s_stream_running)
        return ESP_ERR_INVALID_STATE;

    s_data_cb = cb;
    s_stream_running = true;

    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));
    icm_write_reg(0x4B, 0x02);
    vTaskDelay(pdMS_TO_TICKS(2));
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(2));

    if (xTaskCreatePinnedToCore(icm_dma_worker_task, "icm_dma_task", 4096, NULL,
                                configMAX_PRIORITIES - 1, &s_dma_task_handle, 1) != pdPASS)
    {
        s_stream_running = false;
        s_data_cb = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t drv_icm42688_stop_stream(void)
{
    if (!s_stream_running)
        return ESP_OK;
    s_stream_running = false;

    int timeout = 20;
    while (s_dma_task_handle != NULL && timeout-- > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_data_cb = NULL;
    return ESP_OK;
}
