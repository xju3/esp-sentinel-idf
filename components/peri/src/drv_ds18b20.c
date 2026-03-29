#include "drv_ds18b20.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logger.h"
#include <string.h>

// 私有函数声明
static esp_err_t onewire_reset(void);
static void onewire_write_bit(uint8_t bit);
static uint8_t onewire_read_bit(void);
static void onewire_write_byte(uint8_t byte);
static uint8_t onewire_read_byte(void);
static esp_err_t ds18b20_read_scratchpad(uint8_t *scratchpad);
static esp_err_t ds18b20_write_scratchpad(uint8_t th, uint8_t tl, uint8_t config);
static float ds18b20_raw_to_celsius(int16_t raw_temp);
static uint8_t ds18b20_crc8(const uint8_t *data, size_t len);

// 驱动状态
bool g_ds18b20_initialized = false;
static ds18b20_resolution_t s_current_resolution = DS18B20_RESOLUTION_12BIT;
static uint64_t s_conversion_start_time = 0;
static ds18b20_temp_cb_t s_temp_callback = NULL;

// 1-Wire时序常量（单位：微秒）
#define OW_RESET_PULSE_US   480
#define OW_PRESENCE_WAIT_US 70
#define OW_PRESENCE_TIMEOUT_US 480
#define OW_SLOT_MIN_US      60
#define OW_SLOT_MAX_US      120
#define OW_RECOVERY_US      5
#define DS18B20_TEMP_MIN_C  (-55.0f)
#define DS18B20_TEMP_MAX_C  (125.0f)
#define DS18B20_READ_RETRIES 2

// 延时函数（微秒级）
static void delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // 忙等待
    }
}

// 设置GPIO方向
static void set_gpio_output(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
}

static void set_gpio_input(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
}

// 1-Wire复位脉冲
static esp_err_t onewire_reset(void) {
    esp_err_t ret = ESP_OK;
    
    set_gpio_output();
    gpio_set_level(DS18B20_PIN, 0);
    delay_us(OW_RESET_PULSE_US);
    
    set_gpio_input();
    delay_us(OW_PRESENCE_WAIT_US);
    
    // 检查是否存在设备
    int level = gpio_get_level(DS18B20_PIN);
    if (level != 0) {
        LOG_ERROR("DS18B20 not detected (presence pulse missing)");
        ret = ESP_ERR_NOT_FOUND;
    }
    
    delay_us(OW_PRESENCE_TIMEOUT_US - OW_PRESENCE_WAIT_US);
    return ret;
}

// 写一个位
static void onewire_write_bit(uint8_t bit) {
    if (bit) {
        // 写1：拉低1-15μs，然后释放总线
        set_gpio_output();
        gpio_set_level(DS18B20_PIN, 0);
        delay_us(5);
        set_gpio_input();
        delay_us(OW_SLOT_MAX_US - 5);
    } else {
        // 写0：拉低60-120μs
        set_gpio_output();
        gpio_set_level(DS18B20_PIN, 0);
        delay_us(OW_SLOT_MIN_US);
        set_gpio_input();
        delay_us(OW_SLOT_MAX_US - OW_SLOT_MIN_US);
    }
    delay_us(OW_RECOVERY_US);
}

// 读一个位
static uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;
    
    set_gpio_output();
    gpio_set_level(DS18B20_PIN, 0);
    delay_us(2);
    
    set_gpio_input();
    delay_us(10);
    
    bit = gpio_get_level(DS18B20_PIN);
    delay_us(OW_SLOT_MAX_US - 10 - 2);
    delay_us(OW_RECOVERY_US);
    
    return bit;
}

// 写一个字节
static void onewire_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

// 读一个字节
static uint8_t onewire_read_byte(void) {
    uint8_t byte = 0;
    
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit()) {
            byte |= 0x80;
        }
    }
    
    return byte;
}

// 读取暂存器
static esp_err_t ds18b20_read_scratchpad(uint8_t *scratchpad) {
    if (!scratchpad) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(DS18B20_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_READ_SCRATCHPAD);
    
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = onewire_read_byte();
    }
    
    // 校验 scratchpad CRC（前8字节计算结果应等于第9字节）
    uint8_t crc = ds18b20_crc8(scratchpad, 8);
    if (crc != scratchpad[8]) {
        LOG_WARNF("DS18B20 scratchpad CRC mismatch: calc=0x%02X recv=0x%02X", crc, scratchpad[8]);
        return ESP_ERR_INVALID_CRC;
    }
    
    return ESP_OK;
}

// 写入暂存器
static esp_err_t ds18b20_write_scratchpad(uint8_t th, uint8_t tl, uint8_t config) {
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(DS18B20_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_WRITE_SCRATCHPAD);
    onewire_write_byte(th);
    onewire_write_byte(tl);
    onewire_write_byte(config);
    
    return ESP_OK;
}

// 原始温度值转换为摄氏度
static float ds18b20_raw_to_celsius(int16_t raw_temp) {
    return (float)raw_temp / 16.0f;
}

// Dallas/Maxim CRC-8 (poly 0x31, reflected 0x8C)
static uint8_t ds18b20_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

// 公共函数实现

esp_err_t drv_ds18b20_init(void) {
    if (g_ds18b20_initialized) {
        return ESP_OK;
    }
    
    LOG_DEBUG("Initializing DS18B20 temperature sensor...");
    
    // 配置GPIO
    esp_err_t ret = gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to configure DS18B20 GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
    
    // 尝试复位总线检测设备
    ret = onewire_reset();
    if (ret != ESP_OK) {
        LOG_WARN("DS18B20 not detected on first reset attempt, will retry");
        // 继续初始化，可能在后续操作中检测
    }
    
    // 设置默认分辨率（12位）
    ret = drv_ds18b20_set_resolution(DS18B20_RESOLUTION_12BIT);
    if (ret != ESP_OK) {
        LOG_WARNF("Failed to set default resolution: %s", esp_err_to_name(ret));
        // 继续初始化
    }
    
    g_ds18b20_initialized = true;
    LOG_DEBUG("DS18B20 initialized successfully");
    return ESP_OK;
}

esp_err_t drv_ds18b20_set_resolution(ds18b20_resolution_t resolution) {
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (resolution > DS18B20_RESOLUTION_12BIT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 读取当前配置
    uint8_t scratchpad[9];
    esp_err_t ret = ds18b20_read_scratchpad(scratchpad);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 更新配置寄存器（保留TH和TL）
    uint8_t config = (scratchpad[4] & 0x9F) | (resolution << 5);
    
    ret = ds18b20_write_scratchpad(scratchpad[2], scratchpad[3], config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 保存配置到EEPROM
    ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(DS18B20_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_COPY_SCRATCHPAD);
    
    // 等待复制完成（最大10ms）
    vTaskDelay(pdMS_TO_TICKS(10));
    
    s_current_resolution = resolution;
    LOG_DEBUGF("DS18B20 resolution set to %d-bit", 9 + resolution);
    
    return ESP_OK;
}

esp_err_t drv_ds18b20_start_conversion(void) {
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(DS18B20_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_CONVERT_T);
    
    // 记录转换开始时间
    s_conversion_start_time = esp_timer_get_time();
    
    // 如果使用寄生电源，需要拉高总线供电
    // 这里假设使用外部供电，所以不需要
    
    return ESP_OK;
}

bool drv_ds18b20_is_conversion_done(void) {
    if (!g_ds18b20_initialized || s_conversion_start_time == 0) {
        return false;
    }
    
    // 根据分辨率计算转换时间
    uint32_t conversion_time_ms;
    switch (s_current_resolution) {
        case DS18B20_RESOLUTION_9BIT:
            conversion_time_ms = 94;
            break;
        case DS18B20_RESOLUTION_10BIT:
            conversion_time_ms = 188;
            break;
        case DS18B20_RESOLUTION_11BIT:
            conversion_time_ms = 375;
            break;
        case DS18B20_RESOLUTION_12BIT:
        default:
            conversion_time_ms = 750;
            break;
    }
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed_ms = (current_time - s_conversion_start_time) / 1000;
    
    return elapsed_ms >= conversion_time_ms;
}

esp_err_t drv_ds18b20_read_temperature(float *temperature) {
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (int attempt = 0; attempt < DS18B20_READ_RETRIES; attempt++) {
        if (s_conversion_start_time == 0) {
            esp_err_t start_ret = drv_ds18b20_start_conversion();
            if (start_ret != ESP_OK) {
                return start_ret;
            }
        }

        // 如果转换未完成，等待
        if (!drv_ds18b20_is_conversion_done()) {
            LOG_DEBUG("Waiting for temperature conversion to complete...");

            // 根据分辨率等待相应时间
            uint32_t wait_time_ms;
            switch (s_current_resolution) {
                case DS18B20_RESOLUTION_9BIT:
                    wait_time_ms = 94;
                    break;
                case DS18B20_RESOLUTION_10BIT:
                    wait_time_ms = 188;
                    break;
                case DS18B20_RESOLUTION_11BIT:
                    wait_time_ms = 375;
                    break;
                case DS18B20_RESOLUTION_12BIT:
                default:
                    wait_time_ms = 750;
                    break;
            }

            vTaskDelay(pdMS_TO_TICKS(wait_time_ms));
        }

        uint8_t scratchpad[9];
        esp_err_t ret = ds18b20_read_scratchpad(scratchpad);
        if (ret != ESP_OK) {
            s_conversion_start_time = 0;
            continue;
        }

        int16_t raw_temp = (int16_t)(((uint16_t)scratchpad[1] << 8) | scratchpad[0]);
        float temp_c = ds18b20_raw_to_celsius(raw_temp);
        if (temp_c < DS18B20_TEMP_MIN_C || temp_c > DS18B20_TEMP_MAX_C) {
            LOG_WARNF("DS18B20 temperature out of range: raw=0x%04X temp=%.2f°C",
                      (uint16_t)raw_temp, temp_c);
            s_conversion_start_time = 0;
            continue;
        }

        *temperature = temp_c;
        LOG_DEBUGF("Temperature read: %.2f°C", *temperature);
        s_conversion_start_time = 0;
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t drv_ds18b20_read_temperature_async(ds18b20_temp_cb_t callback) {
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 保存回调函数
    s_temp_callback = callback;
    
    // 启动转换
    esp_err_t ret = drv_ds18b20_start_conversion();
    if (ret != ESP_OK) {
        s_temp_callback = NULL;
        return ret;
    }
    
    // 在实际应用中，这里可以启动一个定时器或任务来检查转换状态
    // 并在转换完成后调用回调函数
    // 由于这是简化实现，用户需要手动调用drv_ds18b20_is_conversion_done()
    // 并在转换完成后调用drv_ds18b20_read_temperature()
    
    return ESP_OK;
}

esp_err_t drv_ds18b20_self_test(void) {
    if (!g_ds18b20_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("Starting DS18B20 self-test...");
    
    // 1. 测试总线复位
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Bus reset error");
        return ret;
    }
    
    // 2. 测试设备存在
    onewire_write_byte(DS18B20_CMD_SKIP_ROM);
    onewire_write_byte(DS18B20_CMD_READ_POWER_SUPPLY);
    
    uint8_t power_mode = onewire_read_bit();
    if (power_mode == 0) {
        LOG_WARN("DS18B20 is in parasite power mode");
    } else {
        LOG_DEBUG("DS18B20 is in external power mode");
    }
    
    // 3. 读取暂存器测试
    uint8_t scratchpad[9];
    ret = ds18b20_read_scratchpad(scratchpad);
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot read scratchpad");
        return ret;
    }
    
    LOG_DEBUGF("Scratchpad read successfully:");
    LOG_DEBUGF("  Temp LSB: 0x%02X", scratchpad[0]);
    LOG_DEBUGF("  Temp MSB: 0x%02X", scratchpad[1]);
    LOG_DEBUGF("  TH: 0x%02X", scratchpad[2]);
    LOG_DEBUGF("  TL: 0x%02X", scratchpad[3]);
    LOG_DEBUGF("  Config: 0x%02X", scratchpad[4]);
    
    // 4. 测试温度转换
    ret = drv_ds18b20_start_conversion();
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot start temperature conversion");
        return ret;
    }
    
    // 等待转换完成
    vTaskDelay(pdMS_TO_TICKS(800)); // 等待最大转换时间
    
    float temperature;
    ret = drv_ds18b20_read_temperature(&temperature);
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot read temperature");
        return ret;
    }
    
    // 检查温度是否在合理范围内
    if (temperature < -55.0f || temperature > 125.0f) {
        LOG_WARNF("Temperature out of expected range: %.2f°C", temperature);
    } else {
        LOG_DEBUGF("Temperature reading valid: %.2f°C", temperature);
    }
    
    LOG_DEBUG("DS18B20 self-test passed");
    return ESP_OK;
}
