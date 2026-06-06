#include "drv_t1820b.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logger.h"

// 私有函数声明
static esp_err_t onewire_reset(void);
static void onewire_write_bit(uint8_t bit);
static uint8_t onewire_read_bit(void);
static void onewire_write_byte(uint8_t byte);
static uint8_t onewire_read_byte(void);
static esp_err_t t1820b_read_scratchpad(uint8_t *scratchpad);
static esp_err_t t1820b_read_temperature_frame(uint8_t *frame);
static esp_err_t t1820b_write_config(const uint8_t *config_bytes, size_t len);
static esp_err_t t1820b_copy_page0(void);
static float t1820b_raw_to_celsius(int16_t raw_temp);
static uint8_t onewire_crc8(const uint8_t *data, size_t len);
static uint32_t t1820b_conversion_time_ms(t1820b_resolution_t resolution);
static uint8_t t1820b_avg_bits_for_resolution(t1820b_resolution_t resolution);

// 驱动状态
bool g_t1820b_initialized = false;
static t1820b_resolution_t s_current_resolution = T1820B_RESOLUTION_9BIT;
static uint64_t s_conversion_start_time = 0;
static t1820b_temp_cb_t s_temp_callback = NULL;

// 1-Wire时序常量（单位：微秒）
#define OW_RESET_PULSE_US        480
#define OW_PRESENCE_WAIT_US      70
#define OW_PRESENCE_TIMEOUT_US   480
#define OW_SLOT_MIN_US           60
#define OW_SLOT_MAX_US           120
#define OW_RECOVERY_US           5

#define T1820B_TEMP_MIN_C        (-103.0f)
#define T1820B_TEMP_MAX_C        (153.0f)
#define T1820B_READ_RETRIES      2
#define T1820B_TEMP_FRAME_BYTES  3
#define T1820B_SCRATCHPAD_BYTES  9
#define T1820B_COPY_PAGE_MS      40

#define T1820B_TEMP_CMD_INDEX    1
#define T1820B_TEMP_CFG_INDEX    2
#define T1820B_ALERT_MODE_INDEX  3
#define T1820B_TH_LSB_INDEX      4
#define T1820B_TH_MSB_INDEX      5
#define T1820B_TL_LSB_INDEX      6
#define T1820B_TL_MSB_INDEX      7

#define T1820B_TEMP_CMD_MEAS_MASK        0xC0
#define T1820B_TEMP_CMD_SINGLE_SHOT      0x40

#define T1820B_TEMP_CFG_MPS_MASK         0xE0
#define T1820B_TEMP_CFG_AVG_MASK         0x18
#define T1820B_TEMP_CFG_AVG_SHIFT        3
#define T1820B_TEMP_CFG_SLEEP_EN         0x01

// 延时函数（微秒级）
static void delay_us(uint32_t us)
{
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us)
    {
    }
}

// 设置 GPIO 方向
static void set_gpio_output(void)
{
    gpio_set_direction(T1820B_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(T1820B_PIN, GPIO_PULLUP_ONLY);
}

static void set_gpio_input(void)
{
    gpio_set_direction(T1820B_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(T1820B_PIN, GPIO_PULLUP_ONLY);
}

// 1-Wire 复位脉冲
static esp_err_t onewire_reset(void)
{
    esp_err_t ret = ESP_OK;

    set_gpio_output();
    gpio_set_level(T1820B_PIN, 0);
    delay_us(OW_RESET_PULSE_US);

    set_gpio_input();
    delay_us(OW_PRESENCE_WAIT_US);

    int level = gpio_get_level(T1820B_PIN);
    if (level != 0)
    {
        LOG_ERROR("T1820B not detected (presence pulse missing)");
        ret = ESP_ERR_NOT_FOUND;
    }

    delay_us(OW_PRESENCE_TIMEOUT_US - OW_PRESENCE_WAIT_US);
    return ret;
}

static void onewire_write_bit(uint8_t bit)
{
    if (bit)
    {
        set_gpio_output();
        gpio_set_level(T1820B_PIN, 0);
        delay_us(5);
        set_gpio_input();
        delay_us(OW_SLOT_MAX_US - 5);
    }
    else
    {
        set_gpio_output();
        gpio_set_level(T1820B_PIN, 0);
        delay_us(OW_SLOT_MIN_US);
        set_gpio_input();
        delay_us(OW_SLOT_MAX_US - OW_SLOT_MIN_US);
    }
    delay_us(OW_RECOVERY_US);
}

static uint8_t onewire_read_bit(void)
{
    uint8_t bit = 0;

    set_gpio_output();
    gpio_set_level(T1820B_PIN, 0);
    delay_us(2);

    set_gpio_input();
    delay_us(10);

    bit = gpio_get_level(T1820B_PIN);
    delay_us(OW_SLOT_MAX_US - 10 - 2);
    delay_us(OW_RECOVERY_US);

    return bit;
}

static void onewire_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        onewire_write_bit(byte & 0x01U);
        byte >>= 1;
    }
}

static uint8_t onewire_read_byte(void)
{
    uint8_t byte = 0;

    for (int i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (onewire_read_bit())
        {
            byte |= 0x80U;
        }
    }

    return byte;
}

static esp_err_t t1820b_read_temperature_frame(uint8_t *frame)
{
    if (frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_READ_TEMPERATURE);

    for (int i = 0; i < T1820B_TEMP_FRAME_BYTES; ++i)
    {
        frame[i] = onewire_read_byte();
    }

    uint8_t crc = onewire_crc8(frame, 2);
    if (crc != frame[2])
    {
        LOG_WARNF("T1820B temperature CRC mismatch: calc=0x%02X recv=0x%02X", crc, frame[2]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t t1820b_read_scratchpad(uint8_t *scratchpad)
{
    if (scratchpad == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_READ_SCRATCHPAD);

    for (int i = 0; i < T1820B_SCRATCHPAD_BYTES; ++i)
    {
        scratchpad[i] = onewire_read_byte();
    }

    uint8_t crc = onewire_crc8(scratchpad, T1820B_SCRATCHPAD_BYTES - 1);
    if (crc != scratchpad[T1820B_SCRATCHPAD_BYTES - 1])
    {
        LOG_WARNF("T1820B scratchpad CRC mismatch: calc=0x%02X recv=0x%02X",
                  crc,
                  scratchpad[T1820B_SCRATCHPAD_BYTES - 1]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t t1820b_write_config(const uint8_t *config_bytes, size_t len)
{
    if (config_bytes == NULL || len != 7U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_WRITE_SCRATCHPAD);
    for (size_t i = 0; i < len; ++i)
    {
        onewire_write_byte(config_bytes[i]);
    }

    return ESP_OK;
}

static esp_err_t t1820b_copy_page0(void)
{
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_COPY_SCRATCHPAD);
    vTaskDelay(pdMS_TO_TICKS(T1820B_COPY_PAGE_MS));
    return ESP_OK;
}

static float t1820b_raw_to_celsius(int16_t raw_temp)
{
    return (float)raw_temp / 256.0f;
}

// Dallas/Maxim CRC-8 (poly 0x31, reflected 0x8C); datasheet polynomial matches
static uint8_t onewire_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01U;
            crc >>= 1;
            if (mix)
            {
                crc ^= 0x8CU;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

static uint32_t t1820b_conversion_time_ms(t1820b_resolution_t resolution)
{
    switch (resolution)
    {
        case T1820B_RESOLUTION_9BIT:
            return 3;
        case T1820B_RESOLUTION_10BIT:
            return 6;
        case T1820B_RESOLUTION_11BIT:
            return 9;
        case T1820B_RESOLUTION_12BIT:
        default:
            return 16;
    }
}

static uint8_t t1820b_avg_bits_for_resolution(t1820b_resolution_t resolution)
{
    switch (resolution)
    {
        case T1820B_RESOLUTION_9BIT:
            return 0x00;
        case T1820B_RESOLUTION_10BIT:
            return 0x01;
        case T1820B_RESOLUTION_11BIT:
            return 0x02;
        case T1820B_RESOLUTION_12BIT:
        default:
            return 0x03;
    }
}

esp_err_t drv_t1820b_init(void)
{
    if (g_t1820b_initialized)
    {
        LOG_DEBUG("T1820B driver already initialized");
        return ESP_OK;
    }

    LOG_DEBUG("Initializing T1820B temperature sensor...");

    esp_err_t ret = gpio_set_direction(T1820B_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to configure T1820B GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_pull_mode(T1820B_PIN, GPIO_PULLUP_ONLY);

    ret = onewire_reset();
    if (ret != ESP_OK)
    {
        LOG_WARN("T1820B not detected during init");
        return ret;
    }

    g_t1820b_initialized = true;
    s_conversion_start_time = 0;
    s_temp_callback = NULL;

    ret = drv_t1820b_set_resolution(T1820B_RESOLUTION_9BIT);
    if (ret != ESP_OK)
    {
        LOG_WARNF("Failed to apply default T1820B averaging profile: %s", esp_err_to_name(ret));
        return ret;
    }

    LOG_DEBUG("T1820B initialized successfully");
    return ESP_OK;
}

esp_err_t drv_t1820b_set_resolution(t1820b_resolution_t resolution)
{
    if (!g_t1820b_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (resolution > T1820B_RESOLUTION_12BIT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t scratchpad[T1820B_SCRATCHPAD_BYTES] = {0};
    esp_err_t ret = t1820b_read_scratchpad(scratchpad);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t temp_cmd = (scratchpad[T1820B_TEMP_CMD_INDEX] & ~T1820B_TEMP_CMD_MEAS_MASK) |
                       T1820B_TEMP_CMD_SINGLE_SHOT;
    uint8_t temp_cfg = scratchpad[T1820B_TEMP_CFG_INDEX] & ~T1820B_TEMP_CFG_AVG_MASK;
    temp_cfg |= (uint8_t)(t1820b_avg_bits_for_resolution(resolution) << T1820B_TEMP_CFG_AVG_SHIFT);
    temp_cfg |= T1820B_TEMP_CFG_SLEEP_EN;

    if (temp_cmd == scratchpad[T1820B_TEMP_CMD_INDEX] &&
        temp_cfg == scratchpad[T1820B_TEMP_CFG_INDEX])
    {
        s_current_resolution = resolution;
        LOG_DEBUGF("T1820B averaging profile already matches compatibility level %d", 9 + resolution);
        return ESP_OK;
    }

    uint8_t config_bytes[7] = {
        temp_cmd,
        temp_cfg,
        scratchpad[T1820B_ALERT_MODE_INDEX],
        scratchpad[T1820B_TH_LSB_INDEX],
        scratchpad[T1820B_TH_MSB_INDEX],
        scratchpad[T1820B_TL_LSB_INDEX],
        scratchpad[T1820B_TL_MSB_INDEX],
    };

    ret = t1820b_write_config(config_bytes, sizeof(config_bytes));
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = t1820b_copy_page0();
    if (ret != ESP_OK)
    {
        return ret;
    }

    s_current_resolution = resolution;
    LOG_DEBUGF("T1820B averaging profile set to compatibility level %d (avg bits=%u)",
               9 + resolution,
               t1820b_avg_bits_for_resolution(resolution));
    return ESP_OK;
}

esp_err_t drv_t1820b_start_conversion(void)
{
    if (!g_t1820b_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_CONVERT_T);
    s_conversion_start_time = esp_timer_get_time();
    return ESP_OK;
}

bool drv_t1820b_is_conversion_done(void)
{
    if (!g_t1820b_initialized || s_conversion_start_time == 0)
    {
        return false;
    }

    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed_ms = (current_time - s_conversion_start_time) / 1000U;
    return elapsed_ms >= t1820b_conversion_time_ms(s_current_resolution);
}

esp_err_t drv_t1820b_read_temperature(float *temperature)
{
    if (!g_t1820b_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (temperature == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < T1820B_READ_RETRIES; ++attempt)
    {
        if (s_conversion_start_time == 0)
        {
            esp_err_t start_ret = drv_t1820b_start_conversion();
            if (start_ret != ESP_OK)
            {
                return start_ret;
            }
        }

        if (!drv_t1820b_is_conversion_done())
        {
            vTaskDelay(pdMS_TO_TICKS(t1820b_conversion_time_ms(s_current_resolution)));
        }

        uint8_t frame[T1820B_TEMP_FRAME_BYTES] = {0};
        esp_err_t ret = t1820b_read_temperature_frame(frame);
        if (ret != ESP_OK)
        {
            s_conversion_start_time = 0;
            continue;
        }

        int16_t raw_temp = (int16_t)(((uint16_t)frame[1] << 8) | frame[0]);
        float temp_c = t1820b_raw_to_celsius(raw_temp);
        if (temp_c < T1820B_TEMP_MIN_C || temp_c > T1820B_TEMP_MAX_C)
        {
            LOG_WARNF("T1820B temperature out of range: raw=0x%04X temp=%.3f°C",
                      (uint16_t)raw_temp,
                      temp_c);
            s_conversion_start_time = 0;
            continue;
        }

        *temperature = temp_c;
        if (s_temp_callback != NULL)
        {
            s_temp_callback(temp_c);
            s_temp_callback = NULL;
        }
        LOG_DEBUGF("T1820B temperature read: %.3f°C", temp_c);
        s_conversion_start_time = 0;
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t drv_t1820b_read_temperature_async(t1820b_temp_cb_t callback)
{
    if (!g_t1820b_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (callback == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_temp_callback = callback;
    esp_err_t ret = drv_t1820b_start_conversion();
    if (ret != ESP_OK)
    {
        s_temp_callback = NULL;
        return ret;
    }

    return ESP_OK;
}

esp_err_t drv_t1820b_self_test(void)
{
    if (!g_t1820b_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    LOG_DEBUG("Starting T1820B self-test...");

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK)
    {
        LOG_ERROR("Self-test failed: bus reset error");
        return ret;
    }

    uint8_t scratchpad[T1820B_SCRATCHPAD_BYTES] = {0};
    ret = t1820b_read_scratchpad(scratchpad);
    if (ret != ESP_OK)
    {
        LOG_ERROR("Self-test failed: cannot read T1820B scratchpad");
        return ret;
    }

    LOG_DEBUGF("T1820B scratchpad status=0x%02X temp_cmd=0x%02X temp_cfg=0x%02X alert=0x%02X",
               scratchpad[0],
               scratchpad[T1820B_TEMP_CMD_INDEX],
               scratchpad[T1820B_TEMP_CFG_INDEX],
               scratchpad[T1820B_ALERT_MODE_INDEX]);

    ret = drv_t1820b_start_conversion();
    if (ret != ESP_OK)
    {
        LOG_ERROR("Self-test failed: cannot start temperature conversion");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(t1820b_conversion_time_ms(s_current_resolution)));

    float temperature = 0.0f;
    ret = drv_t1820b_read_temperature(&temperature);
    if (ret != ESP_OK)
    {
        LOG_ERROR("Self-test failed: cannot read T1820B temperature");
        return ret;
    }

    if (temperature < T1820B_TEMP_MIN_C || temperature > T1820B_TEMP_MAX_C)
    {
        LOG_WARNF("T1820B temperature outside absolute range: %.3f°C", temperature);
    }
    else
    {
        LOG_DEBUGF("T1820B temperature reading valid: %.3f°C", temperature);
    }

    LOG_DEBUG("T1820B self-test passed");
    return ESP_OK;
}
