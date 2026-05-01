#include "drv_t1820b.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logger.h"

// 私有函数声明
static esp_err_t onewire_reset(void);
static esp_err_t onewire_reset_with_pulse(uint32_t reset_pulse_us);
static void onewire_drive_low(void);
static void onewire_release_bus(void);
static void onewire_write_bit(uint8_t bit);
static uint8_t onewire_read_bit(void);
static void onewire_write_byte(uint8_t byte);
static uint8_t onewire_read_byte(void);
static esp_err_t t1820b_send_skip_rom_command(uint8_t command);
static esp_err_t t1820b_read_rom(uint8_t *rom_code);
static esp_err_t t1820b_read_scratchpad(uint8_t *scratchpad);
static esp_err_t t1820b_read_temperature_frame(uint8_t *temp_frame);
static void t1820b_update_conversion_time(uint8_t temp_cfg);
static bool t1820b_wait_for_conversion(void);
static float t1820b_raw_to_celsius(int16_t raw_temp);
static uint8_t t1820b_crc8(const uint8_t *data, size_t len);

// 驱动状态
bool g_t1820b_initialized = false;
static uint64_t s_conversion_start_time = 0;
static uint32_t s_conversion_time_us = 20000;
static t1820b_temp_cb_t s_temp_callback = NULL;

// 1-Wire时序常量（单位：微秒）
#define OW_RESET_PULSE_US 480
#define OW_RESET_PULSE_EXTENDED_US 960
#define OW_RESET_PULSE_LOW_VOLTAGE_US 2000
#define OW_PRESENCE_WAIT_US 15
#define OW_PRESENCE_POLL_STEP_US 5
#define OW_PRESENCE_TIMEOUT_US 480
#define OW_SLOT_MIN_US 60
#define OW_SLOT_MAX_US 120
#define OW_RECOVERY_US 5
#define OW_READ_INIT_US 3
#define OW_READ_SAMPLE_US 9
#define T1820B_TEMP_MIN_C (-103.0f)
#define T1820B_TEMP_MAX_C (153.0f)
#define T1820B_READ_RETRIES 2
#define T1820B_ROM_READ_RETRIES 3
#define T1820B_INIT_SETTLE_MS 2
#define T1820B_CONVERSION_MARGIN_US 2000U
#define T1820B_CONVERSION_TIMEOUT_US 30000ULL
#define T1820B_CONVERSION_POLL_INTERVAL_MS 1

// 延时函数（微秒级）
static void delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // 忙等待
    }
}

static void onewire_drive_low(void) {
    gpio_set_level(T1820B_PIN, 0);
}

static void onewire_release_bus(void) {
    gpio_set_level(T1820B_PIN, 1);
}

// 1-Wire复位脉冲
static esp_err_t onewire_reset(void) {
    static const uint32_t s_reset_pulses[] = {
        OW_RESET_PULSE_US,
        OW_RESET_PULSE_EXTENDED_US,
        OW_RESET_PULSE_LOW_VOLTAGE_US,
    };

    for (size_t i = 0; i < sizeof(s_reset_pulses) / sizeof(s_reset_pulses[0]); i++) {
        esp_err_t ret = onewire_reset_with_pulse(s_reset_pulses[i]);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
    }

    LOG_ERROR("T1820B not detected (presence pulse missing)");
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t onewire_reset_with_pulse(uint32_t reset_pulse_us) {
    esp_err_t ret = ESP_OK;
    
    onewire_drive_low();
    delay_us(reset_pulse_us);
    
    onewire_release_bus();
    delay_us(OW_PRESENCE_WAIT_US);

    // presence pulse: 释放后 15~60us 开始，持续 60~240us。
    // 使用窗口扫描比单点采样更稳健，尤其在边缘供电/布线条件下。
    bool detected = false;
    uint32_t elapsed_us = OW_PRESENCE_WAIT_US;
    while (elapsed_us < OW_PRESENCE_TIMEOUT_US) {
        if (gpio_get_level(T1820B_PIN) == 0) {
            detected = true;
            break;
        }
        delay_us(OW_PRESENCE_POLL_STEP_US);
        elapsed_us += OW_PRESENCE_POLL_STEP_US;
    }

    if (!detected) {
        ret = ESP_ERR_NOT_FOUND;
    }

    if (elapsed_us < OW_PRESENCE_TIMEOUT_US) {
        delay_us(OW_PRESENCE_TIMEOUT_US - elapsed_us);
    }
    return ret;
}

// 写一个位
static void onewire_write_bit(uint8_t bit) {
    if (bit) {
        // 写1：拉低1-15μs，然后释放总线
        onewire_drive_low();
        delay_us(5);
        onewire_release_bus();
        delay_us(OW_SLOT_MAX_US - 5);
    } else {
        // 写0：拉低60-120μs
        onewire_drive_low();
        delay_us(OW_SLOT_MIN_US);
        onewire_release_bus();
        delay_us(OW_SLOT_MAX_US - OW_SLOT_MIN_US);
    }
    delay_us(OW_RECOVERY_US);
}

// 读一个位
static uint8_t onewire_read_bit(void) {
    onewire_drive_low();
    delay_us(OW_READ_INIT_US);

    onewire_release_bus();
    delay_us(OW_READ_SAMPLE_US);

    uint8_t bit = gpio_get_level(T1820B_PIN);
    delay_us(OW_SLOT_MAX_US - OW_READ_INIT_US - OW_READ_SAMPLE_US);
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

static esp_err_t t1820b_send_skip_rom_command(uint8_t command) {
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(command);
    return ESP_OK;
}

// 读取ROM编码
static esp_err_t t1820b_read_rom(uint8_t *rom_code) {
    if (!rom_code) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int attempt = 0; attempt < T1820B_ROM_READ_RETRIES; attempt++) {
        esp_err_t ret = onewire_reset();
        if (ret != ESP_OK) {
            return ret;
        }

        onewire_write_byte(T1820B_CMD_READ_ROM);

        for (int i = 0; i < 8; i++) {
            rom_code[i] = onewire_read_byte();
        }

        uint8_t crc = t1820b_crc8(rom_code, 7);
        if (crc == rom_code[7]) {
            return ESP_OK;
        }

        LOG_WARNF("T1820B ROM CRC mismatch: calc=0x%02X recv=0x%02X raw=%02X %02X %02X %02X %02X %02X %02X %02X",
                  crc, rom_code[7],
                  rom_code[0], rom_code[1], rom_code[2], rom_code[3],
                  rom_code[4], rom_code[5], rom_code[6], rom_code[7]);
    }

    return ESP_ERR_INVALID_CRC;
}

// 读取 Scratchpad：Status(0x03) 到 Crc_scratch(0x0B)
static esp_err_t t1820b_read_scratchpad(uint8_t *scratchpad) {
    if (!scratchpad) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }

    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_READ_SCRATCHPAD);

    for (int i = 0; i < 9; i++) {
        scratchpad[i] = onewire_read_byte();
    }

    uint8_t crc = t1820b_crc8(scratchpad, 8);
    if (crc != scratchpad[8]) {
        LOG_WARNF("T1820B scratchpad CRC mismatch: calc=0x%02X recv=0x%02X",
                  crc, scratchpad[8]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

// 读取温度数据：2字节温度 + 1字节CRC
static esp_err_t t1820b_read_temperature_frame(uint8_t *temp_frame) {
    if (!temp_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_READ_TEMPERATURE);

    for (int i = 0; i < 3; i++) {
        temp_frame[i] = onewire_read_byte();
    }

    uint8_t crc = t1820b_crc8(temp_frame, 2);
    if (crc != temp_frame[2]) {
        LOG_WARNF("T1820B temperature CRC mismatch: calc=0x%02X recv=0x%02X", crc, temp_frame[2]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static void t1820b_update_conversion_time(uint8_t temp_cfg) {
    uint8_t avg_bits = (temp_cfg >> 3) & 0x03;

    switch (avg_bits) {
        case 0x00:
            s_conversion_time_us = 2200U + T1820B_CONVERSION_MARGIN_US;
            break;
        case 0x01:
            s_conversion_time_us = 5200U + T1820B_CONVERSION_MARGIN_US;
            break;
        case 0x02:
            s_conversion_time_us = 8500U + T1820B_CONVERSION_MARGIN_US;
            break;
        case 0x03:
        default:
            s_conversion_time_us = 15300U + T1820B_CONVERSION_MARGIN_US;
            break;
    }
}

static bool t1820b_wait_for_conversion(void) {
    uint64_t deadline = s_conversion_start_time + T1820B_CONVERSION_TIMEOUT_US;

    while (!drv_t1820b_is_conversion_done()) {
        if (esp_timer_get_time() >= deadline) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(T1820B_CONVERSION_POLL_INTERVAL_MS));
    }

    return true;
}

// 原始温度值转换为摄氏度
static float t1820b_raw_to_celsius(int16_t raw_temp) {
    return ((float)raw_temp / 256.0f) + 25.0f;
}

// Dallas/Maxim CRC-8 / T1820B CRC-8 (poly x8 + x5 + x4 + 1)
static uint8_t t1820b_crc8(const uint8_t *data, size_t len) {
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

esp_err_t drv_t1820b_init(void) {
    if (g_t1820b_initialized) {
        return ESP_OK;
    }
    
    LOG_DEBUG("Initializing T1820B temperature sensor...");
    
    // 配置GPIO
    esp_err_t ret = gpio_set_direction(T1820B_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to configure T1820B GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    gpio_set_pull_mode(T1820B_PIN, GPIO_PULLUP_ONLY);
    onewire_release_bus();
    
    // 尝试复位总线检测设备
    ret = onewire_reset();
    if (ret != ESP_OK) {
        LOG_WARN("T1820B not detected on first reset attempt, will retry during reads");
    }

    uint8_t rom_code[8];
    ret = t1820b_read_rom(rom_code);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to read T1820B ROM during init: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = t1820b_send_skip_rom_command(T1820B_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to soft reset T1820B: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(T1820B_INIT_SETTLE_MS));

    uint8_t scratchpad[9];
    ret = t1820b_read_scratchpad(scratchpad);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to read T1820B scratchpad during init: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t status = scratchpad[0];
    uint8_t temp_cmd = scratchpad[1];
    uint8_t temp_cfg = scratchpad[2];
    t1820b_update_conversion_time(temp_cfg);

    if ((status & (1U << 3)) || ((temp_cmd & 0x0F) == 0x0A)) {
        LOG_WARN("T1820B heater state is enabled, sending HEAT_OFF");
        ret = t1820b_send_skip_rom_command(T1820B_CMD_HEAT_OFF);
        if (ret != ESP_OK) {
            LOG_ERRORF("Failed to send T1820B HEAT_OFF: %s", esp_err_to_name(ret));
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        ret = t1820b_read_scratchpad(scratchpad);
        if (ret == ESP_OK) {
            status = scratchpad[0];
            temp_cmd = scratchpad[1];
            temp_cfg = scratchpad[2];
            t1820b_update_conversion_time(temp_cfg);
        }
    }

    LOG_DEBUGF("T1820B config: status=0x%02X temp_cmd=0x%02X temp_cfg=0x%02X tconv=%u us",
               status, temp_cmd, temp_cfg, s_conversion_time_us);
    
    s_conversion_start_time = 0;
    s_temp_callback = NULL;
    g_t1820b_initialized = true;
    LOG_DEBUG("T1820B initialized successfully");
    return ESP_OK;
}

esp_err_t drv_t1820b_start_conversion(void) {
    if (!g_t1820b_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t rom_code[8];
    esp_err_t ret = t1820b_read_rom(rom_code);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = onewire_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    
    onewire_write_byte(T1820B_CMD_SKIP_ROM);
    onewire_write_byte(T1820B_CMD_CONVERT_T);
    
    // 记录转换开始时间
    s_conversion_start_time = esp_timer_get_time();
    
    // 如果使用寄生电源，需要拉高总线供电
    // 这里假设使用外部供电，所以不需要
    
    return ESP_OK;
}

bool drv_t1820b_is_conversion_done(void) {
    if (!g_t1820b_initialized || s_conversion_start_time == 0) {
        return false;
    }

    return (esp_timer_get_time() - s_conversion_start_time) >= s_conversion_time_us;
}

esp_err_t drv_t1820b_read_temperature(float *temperature) {
    if (!g_t1820b_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (int attempt = 0; attempt < T1820B_READ_RETRIES; attempt++) {
        if (s_conversion_start_time == 0) {
            esp_err_t start_ret = drv_t1820b_start_conversion();
            if (start_ret != ESP_OK) {
                return start_ret;
            }
        }

        if (!t1820b_wait_for_conversion()) {
            LOG_WARN("T1820B conversion timed out");
            s_conversion_start_time = 0;
            continue;
        }

        uint8_t temp_frame[3];
        esp_err_t ret = t1820b_read_temperature_frame(temp_frame);
        if (ret != ESP_OK) {
            s_conversion_start_time = 0;
            continue;
        }

        int16_t raw_temp = (int16_t)(((uint16_t)temp_frame[1] << 8) | temp_frame[0]);
        float temp_c = t1820b_raw_to_celsius(raw_temp);
        LOG_DEBUGF("T1820B raw temperature frame: raw=0x%04X lsb=0x%02X msb=0x%02X crc=0x%02X",
                   (uint16_t)raw_temp, temp_frame[0], temp_frame[1], temp_frame[2]);
        if (temp_c < T1820B_TEMP_MIN_C || temp_c > T1820B_TEMP_MAX_C) {
            LOG_WARNF("T1820B temperature out of range: raw=0x%04X temp=%.2f°C",
                      (uint16_t)raw_temp, temp_c);
            s_conversion_start_time = 0;
            continue;
        }

        *temperature = temp_c;
        LOG_DEBUGF("Temperature read: %.2f°C", *temperature);
        s_conversion_start_time = 0;
        if (s_temp_callback) {
            s_temp_callback(*temperature);
            s_temp_callback = NULL;
        }
        return ESP_OK;
    }

    s_temp_callback = NULL;
    return ESP_FAIL;
}

esp_err_t drv_t1820b_read_temperature_async(t1820b_temp_cb_t callback) {
    if (!g_t1820b_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 保存回调函数
    s_temp_callback = callback;
    
    // 启动转换
    esp_err_t ret = drv_t1820b_start_conversion();
    if (ret != ESP_OK) {
        s_temp_callback = NULL;
        return ret;
    }
    
    // 在实际应用中，这里可以启动一个定时器或任务来检查转换状态
    // 并在转换完成后调用回调函数
    // 由于这是简化实现，用户需要手动调用drv_t1820b_is_conversion_done()
    // 并在转换完成后调用drv_t1820b_read_temperature()
    
    return ESP_OK;
}

esp_err_t drv_t1820b_self_test(void) {
    if (!g_t1820b_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("Starting T1820B self-test...");
    
    // 1. 测试总线复位
    esp_err_t ret = onewire_reset();
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Bus reset error");
        return ret;
    }
    
    // 2. 读取ROM并验证CRC
    uint8_t rom_code[8];
    ret = t1820b_read_rom(rom_code);
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot read ROM");
        return ret;
    }
    
    LOG_DEBUGF("ROM read successfully: %02X %02X %02X %02X %02X %02X %02X %02X",
               rom_code[0], rom_code[1], rom_code[2], rom_code[3],
               rom_code[4], rom_code[5], rom_code[6], rom_code[7]);
    
    // 3. 测试温度转换
    ret = drv_t1820b_start_conversion();
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot start temperature conversion");
        return ret;
    }
    
    // 等待转换完成
    if (!t1820b_wait_for_conversion()) {
        s_conversion_start_time = 0;
        LOG_ERROR("Self-test failed: Temperature conversion timeout");
        return ESP_FAIL;
    }
    
    float temperature;
    ret = drv_t1820b_read_temperature(&temperature);
    if (ret != ESP_OK) {
        LOG_ERROR("Self-test failed: Cannot read temperature");
        return ret;
    }
    
    // 检查温度是否在合理范围内
    if (temperature < T1820B_TEMP_MIN_C || temperature > T1820B_TEMP_MAX_C) {
        LOG_WARNF("Temperature out of expected range: %.2f°C", temperature);
    } else {
        LOG_DEBUGF("Temperature reading valid: %.2f°C", temperature);
    }
    
    LOG_DEBUG("T1820B self-test passed");
    return ESP_OK;
}
