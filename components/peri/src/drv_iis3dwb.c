#include "drv_iis3dwb.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "logger.h"
#include <math.h>

#define IIS3DWB_SPI_HOST SPI2_HOST

#define IIS3DWB_ODR_HZ 26667.0f
#define IIS3DWB_FIFO_BDR_XL_26667 0x0A
#define IIS3DWB_FIFO_MODE_BYPASS 0x00
#define IIS3DWB_FIFO_MODE_CONTINUOUS 0x06
#define IIS3DWB_CTRL1_XL_ENABLE 0x05
#define IIS3DWB_CTRL3_C_BDU_IF_INC 0x44

#define IIS3DWB_FIFO_SAMPLE_BYTES 7

#define DMA_CHUNK_SAMPLES 128
#define DMA_RAW_BYTES (DMA_CHUNK_SAMPLES * IIS3DWB_FIFO_SAMPLE_BYTES)

static SemaphoreHandle_t s_spi_mutex = NULL;
static spi_device_handle_t s_spi_handle = NULL;
static iis3dwb_data_cb_t s_data_cb = NULL;

static uint8_t *s_dma_ping_raw = NULL;
static uint8_t *s_dma_pong_raw = NULL;
static iis3dwb_raw_data_t *s_conv_ping = NULL;
static iis3dwb_raw_data_t *s_conv_pong = NULL;

static bool s_initialized = false;
static volatile bool s_stream_running = false;
static TaskHandle_t s_dma_task_handle = NULL;
static float s_current_odr_hz = IIS3DWB_ODR_HZ;
static iis3dwb_fs_t s_current_fs = IIS3DWB_FS_16G;

static esp_err_t iis3dwb_write_reg(uint8_t reg, uint8_t data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {.length = 8, .cmd = reg & 0x7F, .tx_buffer = &data};
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t iis3dwb_read_reg(uint8_t reg, uint8_t *data)
{
    if (!s_spi_mutex)
        return ESP_FAIL;
    spi_transaction_t t = {.length = 8, .cmd = reg | 0x80, .rx_buffer = data};
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static inline int16_t iis3dwb_le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void iis3dwb_unpack_fifo(const uint8_t *raw, iis3dwb_raw_data_t *out, size_t count)
{
    const uint8_t *p = raw;
    for (size_t i = 0; i < count; i++)
    {
        out[i].header = p[0];
        out[i].x = iis3dwb_le16(p + 1);
        out[i].y = iis3dwb_le16(p + 3);
        out[i].z = iis3dwb_le16(p + 5);
        out[i].temp = 0;
        p += IIS3DWB_FIFO_SAMPLE_BYTES;
    }
}

static float IIS3DWB_Config_ODR_Callback(float ideal_odr)
{
    (void)ideal_odr;
    if (!s_spi_handle)
        return 0.0f;

    iis3dwb_write_reg(IIS3DWB_REG_CTRL3_C, IIS3DWB_CTRL3_C_BDU_IF_INC);

    uint8_t ctrl1 = (uint8_t)((IIS3DWB_CTRL1_XL_ENABLE << 5) | ((uint8_t)s_current_fs << 2));
    iis3dwb_write_reg(IIS3DWB_REG_CTRL1_XL, ctrl1);

    s_current_odr_hz = IIS3DWB_ODR_HZ;
    return s_current_odr_hz;
}

SensorDriver_t iis3dwb_driver = {
    .name = "IIS3DWB",
    .config_hardware_odr = IIS3DWB_Config_ODR_Callback,
};

esp_err_t drv_iis3dwb_init(void)
{
    if (s_initialized)
        return ESP_OK;

    if (s_spi_mutex == NULL)
        s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex)
        return ESP_ERR_NO_MEM;

    spi_bus_config_t buscfg = {
        .miso_io_num = IIS3DWB_PIN_NUM_SDO,
        .mosi_io_num = IIS3DWB_PIN_NUM_SDA,
        .sclk_io_num = IIS3DWB_PIN_NUM_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    esp_err_t ret = spi_bus_initialize(IIS3DWB_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        LOG_ERRORF("drv_iis3dwb_init: spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = IIS3DWB_PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 8,
    };
    ret = spi_bus_add_device(IIS3DWB_SPI_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK)
        return ret;

    s_dma_ping_raw = (uint8_t *)heap_caps_aligned_alloc(32, DMA_RAW_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_dma_pong_raw = (uint8_t *)heap_caps_aligned_alloc(32, DMA_RAW_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dma_ping_raw || !s_dma_pong_raw)
        return ESP_ERR_NO_MEM;

    s_conv_ping = (iis3dwb_raw_data_t *)heap_caps_malloc(sizeof(iis3dwb_raw_data_t) * DMA_CHUNK_SAMPLES,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_conv_pong = (iis3dwb_raw_data_t *)heap_caps_malloc(sizeof(iis3dwb_raw_data_t) * DMA_CHUNK_SAMPLES,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_conv_ping || !s_conv_pong)
        return ESP_ERR_NO_MEM;

    uint8_t who_am_i = 0;
    iis3dwb_read_reg(IIS3DWB_REG_WHO_AM_I, &who_am_i);
    LOG_DEBUGF("IIS3DWB WHO_AM_I: 0x%02x", who_am_i);
    if (who_am_i != IIS3DWB_WHO_AM_I_VAL)
        return ESP_ERR_NOT_FOUND;

    iis3dwb_write_reg(IIS3DWB_REG_CTRL3_C, IIS3DWB_CTRL3_C_BDU_IF_INC);

    s_initialized = true;
    return ESP_OK;
}

esp_err_t drv_iis3dwb_config(const iis3dwb_cfg_t *cfg)
{
    if (!cfg || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;

    s_current_fs = cfg->fs;
    uint8_t ctrl1 = (uint8_t)((IIS3DWB_CTRL1_XL_ENABLE << 5) | ((uint8_t)cfg->fs << 2));
    iis3dwb_write_reg(IIS3DWB_REG_CTRL1_XL, ctrl1);

    iis3dwb_write_reg(IIS3DWB_REG_CTRL3_C, IIS3DWB_CTRL3_C_BDU_IF_INC);

    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL1, 0x00);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL2, 0x00);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL3, IIS3DWB_FIFO_BDR_XL_26667);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_CONTINUOUS);

    s_current_odr_hz = IIS3DWB_ODR_HZ;
    return ESP_OK;
}

static void iis3dwb_dma_worker_task(void *arg)
{
    (void)arg;
    spi_transaction_t trans[2] = {0};
    trans[0].length = DMA_RAW_BYTES * 8;
    trans[0].rx_buffer = s_dma_ping_raw;
    trans[0].cmd = IIS3DWB_REG_FIFO_DATA_OUT_TAG | 0x80;
    trans[1].length = DMA_RAW_BYTES * 8;
    trans[1].rx_buffer = s_dma_pong_raw;
    trans[1].cmd = IIS3DWB_REG_FIFO_DATA_OUT_TAG | 0x80;

    uint8_t ping_pong_flag = 0;

    while (s_stream_running)
    {
        float odr = (s_current_odr_hz > 0.0f) ? s_current_odr_hz : IIS3DWB_ODR_HZ;
        float chunk_ms = ((float)DMA_CHUNK_SAMPLES * 1000.0f) / odr;
        if (chunk_ms < 1.0f)
            chunk_ms = 1.0f;
        vTaskDelay(pdMS_TO_TICKS((uint32_t)ceilf(chunk_ms)));

        spi_transaction_t *t = (ping_pong_flag == 0) ? &trans[0] : &trans[1];
        ping_pong_flag = !ping_pong_flag;

        spi_device_queue_trans(s_spi_handle, t, portMAX_DELAY);

        spi_transaction_t *p_trans = NULL;
        if (spi_device_get_trans_result(s_spi_handle, &p_trans, portMAX_DELAY) == ESP_OK)
        {
            if (s_stream_running && s_data_cb)
            {
                const uint8_t *raw = (const uint8_t *)p_trans->rx_buffer;
                iis3dwb_raw_data_t *dst = (raw == s_dma_ping_raw) ? s_conv_ping : s_conv_pong;
                iis3dwb_unpack_fifo(raw, dst, DMA_CHUNK_SAMPLES);
                s_data_cb(dst, DMA_CHUNK_SAMPLES);
            }
        }
    }

    spi_transaction_t *p_trans = NULL;
    while (spi_device_get_trans_result(s_spi_handle, &p_trans, 0) == ESP_OK)
    {
    }
    s_dma_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t drv_iis3dwb_start_stream(iis3dwb_data_cb_t cb)
{
    if (!cb || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;
    if (s_stream_running)
        return ESP_ERR_INVALID_STATE;

    s_data_cb = cb;
    s_stream_running = true;

    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_BYPASS);
    vTaskDelay(pdMS_TO_TICKS(2));
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL1, 0x00);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL2, 0x00);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL3, IIS3DWB_FIFO_BDR_XL_26667);
    iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_CONTINUOUS);

    if (xTaskCreatePinnedToCore(iis3dwb_dma_worker_task, "iis3dwb_dma_task", 4096, NULL,
                                configMAX_PRIORITIES - 1, &s_dma_task_handle, 1) != pdPASS)
    {
        s_stream_running = false;
        s_data_cb = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t drv_iis3dwb_stop_stream(void)
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
