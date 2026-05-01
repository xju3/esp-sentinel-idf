#include "drv_iis3dwb.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "logger.h"
#include "sdkconfig.h"
#include <math.h>

#define IIS3DWB_SPI_HOST SPI2_HOST

#define IIS3DWB_SPI_DEFAULT_HZ (10 * 1000 * 1000)
#define IIS3DWB_SPI_PROBE_HZ   (200 * 1000)

#define IIS3DWB_ODR_HZ 26667.0f
#define IIS3DWB_FIFO_BDR_XL_26667 0x0A
#define IIS3DWB_FIFO_MODE_BYPASS 0x00
#define IIS3DWB_FIFO_MODE_CONTINUOUS 0x06
#define IIS3DWB_FIFO_WTM_64 0x40
#define IIS3DWB_CTRL1_XL_ODR_26667 0xA0
#define IIS3DWB_CTRL3_C_BDU_IF_INC 0x44
#define IIS3DWB_CTRL4_C_DRDY_MASK 0x04

#define IIS3DWB_SPI_BOOTSTRAP_HOLD_MS 10
#define IIS3DWB_POST_RELEASE_SETTLE_MS 20
#define IIS3DWB_FIFO_START_SETTLE_MS 20

#define IIS3DWB_FIFO_SAMPLE_BYTES 7

#define DMA_CHUNK_SAMPLES 128
#define DMA_RAW_BYTES (DMA_CHUNK_SAMPLES * IIS3DWB_FIFO_SAMPLE_BYTES)

static SemaphoreHandle_t s_spi_mutex = NULL;
static spi_device_handle_t s_spi_handle = NULL;
static imu_data_cb_t s_data_cb = NULL;
static imu_data_cb_ctx_t s_data_cb_ctx = NULL;
static void *s_data_cb_user_ctx = NULL;

static uint8_t *s_dma_ping_raw = NULL;
static uint8_t *s_dma_pong_raw = NULL;
static imu_raw_data_t *s_conv_ping = NULL;
static imu_raw_data_t *s_conv_pong = NULL;

static bool s_spi_resources_ready = false;
static bool g_iis3dwb_initialized = false;
static volatile bool s_stream_running = false;
static TaskHandle_t s_dma_task_handle = NULL;
static float s_current_odr_hz = IIS3DWB_ODR_HZ;
static iis3dwb_fs_t s_current_fs = IIS3DWB_FS_16G;
static bool s_dma_capture_logged = false;

iis3dwb_cfg_t iis3dwb_accel_fs_cfg_16 = {
    .fs = IIS3DWB_FS_16G,
};

iis3dwb_cfg_t iis3dwb_accel_fs_cfg_2 = {
    .fs = IIS3DWB_FS_2G,
};

static esp_err_t iis3dwb_transmit(spi_transaction_t *t)
{
    if (!s_spi_mutex || !s_spi_handle || !t)
        return ESP_FAIL;
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_spi_handle, t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t iis3dwb_write_reg_with_freq(uint8_t reg, uint8_t data, uint32_t freq_hz)
{
    spi_transaction_t t = {
        .length = 8,
        .cmd = reg & 0x7F,
        .tx_buffer = &data,
        .override_freq_hz = freq_hz,
    };
    return iis3dwb_transmit(&t);
}

static esp_err_t iis3dwb_write_reg(uint8_t reg, uint8_t data)
{
    return iis3dwb_write_reg_with_freq(reg, data, IIS3DWB_SPI_DEFAULT_HZ);
}

static esp_err_t iis3dwb_read_bytes_with_freq(uint8_t reg, uint8_t *data, size_t len, uint32_t freq_hz)
{
    if (!data || len == 0U)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .length = len * 8U,
        .cmd = reg | 0x80,
        .rx_buffer = data,
        .override_freq_hz = freq_hz,
    };
    return iis3dwb_transmit(&t);
}

static esp_err_t iis3dwb_read_reg_with_freq(uint8_t reg, uint8_t *data, uint32_t freq_hz)
{
    return iis3dwb_read_bytes_with_freq(reg, data, 1U, freq_hz);
}

static esp_err_t iis3dwb_read_reg(uint8_t reg, uint8_t *data)
{
    return iis3dwb_read_reg_with_freq(reg, data, IIS3DWB_SPI_DEFAULT_HZ);
}

static uint32_t iis3dwb_post_release_settle_ms(void)
{
    return (CONFIG_SENTINEL_SENSOR_POWER_SETTLE_MS > IIS3DWB_POST_RELEASE_SETTLE_MS)
               ? CONFIG_SENTINEL_SENSOR_POWER_SETTLE_MS
               : IIS3DWB_POST_RELEASE_SETTLE_MS;
}

static esp_err_t iis3dwb_hold_sdo_low_during_bootstrap(void)
{
    gpio_config_t drive_low_cfg = {
        .pin_bit_mask = 1ULL << IIS3DWB_PIN_NUM_SDO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&drive_low_cfg);
    if (ret != ESP_OK)
        return ret;

    ret = gpio_set_level(IIS3DWB_PIN_NUM_SDO, 0);
    if (ret != ESP_OK)
        return ret;

    LOG_INFOF("IIS3DWB bootstrap: hold SDO/MISO(GPIO%d) low for %d ms",
              IIS3DWB_PIN_NUM_SDO,
              IIS3DWB_SPI_BOOTSTRAP_HOLD_MS);
    vTaskDelay(pdMS_TO_TICKS(IIS3DWB_SPI_BOOTSTRAP_HOLD_MS));

    gpio_config_t release_cfg = {
        .pin_bit_mask = 1ULL << IIS3DWB_PIN_NUM_SDO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&release_cfg);
    if (ret != ESP_OK)
        return ret;

    uint32_t settle_ms = iis3dwb_post_release_settle_ms();
    LOG_INFOF("IIS3DWB bootstrap: release SDO/MISO and wait %lu ms before SPI probe",
              (unsigned long)settle_ms);
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
    return ESP_OK;
}

static esp_err_t iis3dwb_init_spi_resources(void)
{
    if (s_spi_mutex == NULL)
        s_spi_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex)
        return ESP_ERR_NO_MEM;

    if (s_spi_resources_ready)
        return ESP_OK;

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

    if (!s_spi_handle)
    {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = IIS3DWB_SPI_DEFAULT_HZ,
            .mode = 3,
            .spics_io_num = IIS3DWB_PIN_NUM_CS,
            .queue_size = 7,
            .command_bits = 8,
        };
        ret = spi_bus_add_device(IIS3DWB_SPI_HOST, &devcfg, &s_spi_handle);
        if (ret != ESP_OK)
            return ret;
    }

    if (!s_dma_ping_raw)
        s_dma_ping_raw = (uint8_t *)heap_caps_aligned_alloc(32, DMA_RAW_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dma_pong_raw)
        s_dma_pong_raw = (uint8_t *)heap_caps_aligned_alloc(32, DMA_RAW_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dma_ping_raw || !s_dma_pong_raw)
        return ESP_ERR_NO_MEM;

    if (!s_conv_ping)
    {
        s_conv_ping = (imu_raw_data_t *)heap_caps_malloc(sizeof(imu_raw_data_t) * DMA_CHUNK_SAMPLES,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_conv_pong)
    {
        s_conv_pong = (imu_raw_data_t *)heap_caps_malloc(sizeof(imu_raw_data_t) * DMA_CHUNK_SAMPLES,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!s_conv_ping || !s_conv_pong)
        return ESP_ERR_NO_MEM;

    s_spi_resources_ready = true;
    return ESP_OK;
}

static esp_err_t iis3dwb_probe_identity(void)
{
    uint8_t who_am_i[3] = {0};
    for (size_t i = 0; i < 3; ++i)
    {
        esp_err_t ret = iis3dwb_read_reg_with_freq(IIS3DWB_REG_WHO_AM_I, &who_am_i[i], IIS3DWB_SPI_PROBE_HZ);
        if (ret != ESP_OK)
            return ret;
    }

    LOG_INFOF("IIS3DWB WHO_AM_I x3 @ %d Hz: 0x%02x 0x%02x 0x%02x",
              IIS3DWB_SPI_PROBE_HZ,
              who_am_i[0],
              who_am_i[1],
              who_am_i[2]);

    for (size_t i = 0; i < 3; ++i)
    {
        if (who_am_i[i] != IIS3DWB_WHO_AM_I_VAL)
        {
            LOG_ERRORF("IIS3DWB WHO_AM_I probe failed: got 0x%02x, expected 0x%02x",
                       who_am_i[i],
                       IIS3DWB_WHO_AM_I_VAL);
            return ESP_ERR_NOT_FOUND;
        }
    }

    return ESP_OK;
}

static esp_err_t iis3dwb_apply_ctrl_regs(iis3dwb_fs_t fs)
{
    uint8_t ctrl1 = (uint8_t)(IIS3DWB_CTRL1_XL_ODR_26667 | ((uint8_t)fs << 2));

    esp_err_t ret = iis3dwb_write_reg(IIS3DWB_REG_CTRL1_XL, ctrl1);
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_write_reg(IIS3DWB_REG_CTRL3_C, IIS3DWB_CTRL3_C_BDU_IF_INC);
    if (ret != ESP_OK)
        return ret;

    return iis3dwb_write_reg(IIS3DWB_REG_CTRL4_C, IIS3DWB_CTRL4_C_DRDY_MASK);
}

static esp_err_t iis3dwb_log_config_readback(iis3dwb_fs_t fs)
{
    uint8_t ctrl1 = 0;
    uint8_t ctrl3 = 0;
    uint8_t ctrl4 = 0;
    uint8_t fifo1 = 0;
    uint8_t fifo2 = 0;
    uint8_t fifo3 = 0;
    uint8_t fifo4 = 0;

    esp_err_t ret = iis3dwb_read_reg(IIS3DWB_REG_CTRL1_XL, &ctrl1);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_CTRL3_C, &ctrl3);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_CTRL4_C, &ctrl4);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_CTRL1, &fifo1);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_CTRL2, &fifo2);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_CTRL3, &fifo3);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_CTRL4, &fifo4);
    if (ret != ESP_OK)
        return ret;

    LOG_INFOF("IIS3DWB config readback(fs=%u): CTRL1_XL=0x%02x CTRL3_C=0x%02x CTRL4_C=0x%02x FIFO1=0x%02x FIFO2=0x%02x FIFO3=0x%02x FIFO4=0x%02x",
              (unsigned int)fs,
              ctrl1,
              ctrl3,
              ctrl4,
              fifo1,
              fifo2,
              fifo3,
              fifo4);
    return ESP_OK;
}

static esp_err_t iis3dwb_arm_fifo(void)
{
    esp_err_t ret = iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_BYPASS);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(2));

    ret = iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL1, IIS3DWB_FIFO_WTM_64);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL2, 0x00);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL3, IIS3DWB_FIFO_BDR_XL_26667);
    if (ret != ESP_OK)
        return ret;
    return iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_CONTINUOUS);
}

static esp_err_t iis3dwb_log_fifo_runtime_state(void)
{
    uint8_t fifo_status1 = 0;
    uint8_t fifo_status2 = 0;
    uint8_t fifo_sample[IIS3DWB_FIFO_SAMPLE_BYTES] = {0};

    esp_err_t ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_STATUS1, &fifo_status1);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_reg(IIS3DWB_REG_FIFO_STATUS2, &fifo_status2);
    if (ret != ESP_OK)
        return ret;
    ret = iis3dwb_read_bytes_with_freq(IIS3DWB_REG_FIFO_DATA_OUT_TAG,
                                       fifo_sample,
                                       sizeof(fifo_sample),
                                       IIS3DWB_SPI_DEFAULT_HZ);
    if (ret != ESP_OK)
        return ret;

    LOG_INFOF("IIS3DWB FIFO status: STATUS1=0x%02x STATUS2=0x%02x TAG=0x%02x",
              fifo_status1,
              fifo_status2,
              fifo_sample[0]);
    return ESP_OK;
}

static inline int16_t iis3dwb_le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void iis3dwb_unpack_fifo(const uint8_t *raw, imu_raw_data_t *out, size_t count)
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

    if (iis3dwb_apply_ctrl_regs(s_current_fs) != ESP_OK)
        return 0.0f;

    s_current_odr_hz = IIS3DWB_ODR_HZ;
    return s_current_odr_hz;
}

SensorDriver_t iis3dwb_driver = {
    .name = "IIS3DWB",
    .config_hardware_odr = IIS3DWB_Config_ODR_Callback,
};

esp_err_t drv_iis3dwb_init(void)
{
    if (g_iis3dwb_initialized)
        return ESP_OK;

    esp_err_t ret = iis3dwb_init_spi_resources();
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_hold_sdo_low_during_bootstrap();
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_probe_identity();
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_write_reg(IIS3DWB_REG_CTRL3_C, IIS3DWB_CTRL3_C_BDU_IF_INC);
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_write_reg(IIS3DWB_REG_CTRL4_C, IIS3DWB_CTRL4_C_DRDY_MASK);
    if (ret != ESP_OK)
        return ret;

    g_iis3dwb_initialized = true;
    return ESP_OK;
}

esp_err_t drv_iis3dwb_config(const iis3dwb_cfg_t *cfg)
{
    if (!cfg || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;

    s_current_fs = cfg->fs;
    esp_err_t ret = iis3dwb_apply_ctrl_regs(cfg->fs);
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_arm_fifo();
    if (ret != ESP_OK)
        return ret;

    ret = iis3dwb_log_config_readback(cfg->fs);
    if (ret != ESP_OK)
        return ret;

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

        esp_err_t ret = spi_device_queue_trans(s_spi_handle, t, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            LOG_ERRORF("IIS3DWB DMA queue failed: %s", esp_err_to_name(ret));
            break;
        }

        spi_transaction_t *p_trans = NULL;
        ret = spi_device_get_trans_result(s_spi_handle, &p_trans, portMAX_DELAY);
        if (ret == ESP_OK)
        {
            if (s_stream_running && (s_data_cb || s_data_cb_ctx))
            {
                const uint8_t *raw = (const uint8_t *)p_trans->rx_buffer;
                imu_raw_data_t *dst = (raw == s_dma_ping_raw) ? s_conv_ping : s_conv_pong;
                iis3dwb_unpack_fifo(raw, dst, DMA_CHUNK_SAMPLES);
                if (!s_dma_capture_logged)
                {
                    LOG_INFOF("IIS3DWB DMA/FIFO capture success: first chunk %u samples", DMA_CHUNK_SAMPLES);
                    s_dma_capture_logged = true;
                }
                if (s_data_cb_ctx)
                {
                    s_data_cb_ctx(dst, DMA_CHUNK_SAMPLES, s_data_cb_user_ctx);
                }
                else if (s_data_cb)
                {
                    s_data_cb(dst, DMA_CHUNK_SAMPLES);
                }
            }
        }
        else
        {
            LOG_ERRORF("IIS3DWB DMA result fetch failed: %s", esp_err_to_name(ret));
            break;
        }
    }

    spi_transaction_t *p_trans = NULL;
    while (spi_device_get_trans_result(s_spi_handle, &p_trans, 0) == ESP_OK)
    {
    }
    s_dma_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t drv_iis3dwb_start_stream(imu_data_cb_t cb)
{
    if (!cb || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;
    if (s_stream_running)
        return ESP_ERR_INVALID_STATE;

    s_data_cb = cb;
    s_data_cb_ctx = NULL;
    s_data_cb_user_ctx = NULL;
    s_stream_running = true;
    s_dma_capture_logged = false;

    esp_err_t ret = iis3dwb_arm_fifo();
    if (ret != ESP_OK)
    {
        s_stream_running = false;
        s_data_cb = NULL;
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(IIS3DWB_FIFO_START_SETTLE_MS));
    ret = iis3dwb_log_fifo_runtime_state();
    if (ret != ESP_OK)
    {
        s_stream_running = false;
        s_data_cb = NULL;
        return ret;
    }

    if (xTaskCreatePinnedToCore(iis3dwb_dma_worker_task, "iis3dwb_dma_task", 4096, NULL,
                                configMAX_PRIORITIES - 1, &s_dma_task_handle, 1) != pdPASS)
    {
        s_stream_running = false;
        s_data_cb = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t drv_iis3dwb_start_stream_ex(imu_data_cb_ctx_t cb, void *user_ctx)
{
    if (!cb || !s_spi_handle)
        return ESP_ERR_INVALID_ARG;
    if (s_stream_running)
        return ESP_ERR_INVALID_STATE;

    s_data_cb_ctx = cb;
    s_data_cb_user_ctx = user_ctx;
    s_data_cb = NULL;
    s_stream_running = true;
    s_dma_capture_logged = false;

    esp_err_t ret = iis3dwb_arm_fifo();
    if (ret != ESP_OK)
    {
        s_stream_running = false;
        s_data_cb_ctx = NULL;
        s_data_cb_user_ctx = NULL;
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(IIS3DWB_FIFO_START_SETTLE_MS));
    ret = iis3dwb_log_fifo_runtime_state();
    if (ret != ESP_OK)
    {
        s_stream_running = false;
        s_data_cb_ctx = NULL;
        s_data_cb_user_ctx = NULL;
        return ret;
    }

    if (xTaskCreatePinnedToCore(iis3dwb_dma_worker_task, "iis3dwb_dma_task", 4096, NULL,
                                configMAX_PRIORITIES - 1, &s_dma_task_handle, 1) != pdPASS)
    {
        s_stream_running = false;
        s_data_cb_ctx = NULL;
        s_data_cb_user_ctx = NULL;
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
    s_data_cb_ctx = NULL;
    s_data_cb_user_ctx = NULL;
    return ESP_OK;
}

esp_err_t drv_iis3dwb_enter_standby(void)
{
    if (!g_iis3dwb_initialized || !s_spi_handle)
    {
        return ESP_OK;
    }

    esp_err_t ret = drv_iis3dwb_stop_stream();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = iis3dwb_write_reg(IIS3DWB_REG_FIFO_CTRL4, IIS3DWB_FIFO_MODE_BYPASS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = iis3dwb_write_reg(IIS3DWB_REG_CTRL1_XL, 0x00);
    g_iis3dwb_initialized = false;
    if (ret == ESP_OK)
    {
        LOG_DEBUG("IIS3DWB entered standby");
    }
    return ret;
}
