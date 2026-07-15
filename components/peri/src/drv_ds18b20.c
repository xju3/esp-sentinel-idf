#include "drv_ds18b20.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "logger.h"

static esp_err_t onewire_reset(void);
static void onewire_write_bit(uint8_t bit);
static uint8_t onewire_read_bit(void);
static void onewire_write_byte(uint8_t byte);
static uint8_t onewire_read_byte(void);
static esp_err_t ds18b20_read_scratchpad(uint8_t *scratchpad);
static esp_err_t ds18b20_write_scratchpad(uint8_t th, uint8_t tl,
                                          uint8_t config);
static esp_err_t ds18b20_copy_scratchpad(void);
static float ds18b20_raw_to_celsius(int16_t raw_temp);
static uint8_t onewire_crc8(const uint8_t *data, size_t len);
static uint32_t ds18b20_conversion_time_ms(ds18b20_resolution_t resolution);
static uint8_t ds18b20_config_for_resolution(ds18b20_resolution_t resolution);
static ds18b20_resolution_t ds18b20_resolution_from_config(uint8_t config);

bool g_ds18b20_initialized = false;
static ds18b20_resolution_t s_current_resolution = DS18B20_RESOLUTION_12BIT;
static ds18b20_state_t s_state = DS18B20_STATE_IDLE;
static int64_t s_conversion_start_us = 0;
static StaticSemaphore_t s_mutex_storage;
static SemaphoreHandle_t s_mutex = NULL;

#define OW_RESET_PULSE_US 480
#define OW_PRESENCE_WAIT_US 70
#define OW_PRESENCE_TIMEOUT_US 480
#define OW_SLOT_MIN_US 60
#define OW_SLOT_MAX_US 120
#define OW_RECOVERY_US 5

#define DS18B20_TEMP_MIN_C (-55.0f)
#define DS18B20_TEMP_MAX_C (125.0f)
#define DS18B20_POWER_ON_DEFAULT_RAW 0x0550
#define DS18B20_MAX_READ_RETRIES 2
#define DS18B20_CRC_RETRY_DELAY_MS 10
#define DS18B20_SCRATCHPAD_BYTES 9
#define DS18B20_COPY_EEPROM_MS 10

#define DS18B20_TEMP_LSB_INDEX 0
#define DS18B20_TEMP_MSB_INDEX 1
#define DS18B20_TH_INDEX 2
#define DS18B20_TL_INDEX 3
#define DS18B20_CONFIG_INDEX 4
#define DS18B20_CRC_INDEX 8

#define DS18B20_CONFIG_RESERVED 0x1FU
#define DS18B20_CONFIG_R0 0x20U
#define DS18B20_CONFIG_R1 0x40U
#define DS18B20_CONFIG_RES_MASK (DS18B20_CONFIG_R1 | DS18B20_CONFIG_R0)

static void delay_us(uint32_t us) {
  uint64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
  }
}

static void set_gpio_output(void) {
  gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
}

static void set_gpio_input(void) {
  gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
}

static void set_gpio_strong_high(void) {
  gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DS18B20_PIN, 1);
}

static esp_err_t onewire_reset(void) {
  set_gpio_output();
  gpio_set_level(DS18B20_PIN, 0);
  delay_us(OW_RESET_PULSE_US);

  set_gpio_input();
  delay_us(OW_PRESENCE_WAIT_US);

  int level = gpio_get_level(DS18B20_PIN);
  if (level != 0) {
    LOG_ERROR("DS18B20 not detected (presence pulse missing)");
    delay_us(OW_PRESENCE_TIMEOUT_US - OW_PRESENCE_WAIT_US);
    return ESP_ERR_NOT_FOUND;
  }

  delay_us(OW_PRESENCE_TIMEOUT_US - OW_PRESENCE_WAIT_US);
  return ESP_OK;
}

static void onewire_write_bit(uint8_t bit) {
  set_gpio_output();
  gpio_set_level(DS18B20_PIN, 0);

  if (bit) {
    delay_us(5);
    set_gpio_input();
    delay_us(OW_SLOT_MAX_US - 5);
  } else {
    delay_us(OW_SLOT_MIN_US);
    set_gpio_input();
    delay_us(OW_SLOT_MAX_US - OW_SLOT_MIN_US);
  }

  delay_us(OW_RECOVERY_US);
}

static uint8_t onewire_read_bit(void) {
  set_gpio_output();
  gpio_set_level(DS18B20_PIN, 0);
  delay_us(2);

  set_gpio_input();
  delay_us(10);

  uint8_t bit = (uint8_t)gpio_get_level(DS18B20_PIN);
  delay_us(OW_SLOT_MAX_US - 10 - 2);
  delay_us(OW_RECOVERY_US);

  return bit;
}

static void onewire_write_byte(uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    onewire_write_bit(byte & 0x01U);
    byte >>= 1;
  }
}

static uint8_t onewire_read_byte(void) {
  uint8_t byte = 0;

  for (int i = 0; i < 8; i++) {
    byte >>= 1;
    if (onewire_read_bit()) {
      byte |= 0x80U;
    }
  }

  return byte;
}

static esp_err_t ds18b20_read_scratchpad(uint8_t *scratchpad) {
  if (scratchpad == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = onewire_reset();
  if (ret != ESP_OK) {
    return ret;
  }

  onewire_write_byte(DS18B20_CMD_SKIP_ROM);
  onewire_write_byte(DS18B20_CMD_READ_SCRATCHPAD);

  for (int i = 0; i < DS18B20_SCRATCHPAD_BYTES; ++i) {
    scratchpad[i] = onewire_read_byte();
  }

  uint8_t crc = onewire_crc8(scratchpad, DS18B20_SCRATCHPAD_BYTES - 1);
  if (crc != scratchpad[DS18B20_CRC_INDEX]) {
    LOG_WARNF("DS18B20 scratchpad CRC mismatch: calc=0x%02X recv=0x%02X", crc,
              scratchpad[DS18B20_CRC_INDEX]);
    LOG_WARNF("DS18B20 scratchpad: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
              scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3],
              scratchpad[4], scratchpad[5], scratchpad[6], scratchpad[7],
              scratchpad[8]);
    return ESP_ERR_INVALID_CRC;
  }

  return ESP_OK;
}

static esp_err_t ds18b20_write_scratchpad(uint8_t th, uint8_t tl,
                                          uint8_t config) {
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

static esp_err_t ds18b20_copy_scratchpad(void) {
  esp_err_t ret = onewire_reset();
  if (ret != ESP_OK) {
    return ret;
  }

  onewire_write_byte(DS18B20_CMD_SKIP_ROM);
  onewire_write_byte(DS18B20_CMD_COPY_SCRATCHPAD);
  set_gpio_strong_high();
  vTaskDelay(pdMS_TO_TICKS(DS18B20_COPY_EEPROM_MS));
  set_gpio_input();
  return ESP_OK;
}

static float ds18b20_raw_to_celsius(int16_t raw_temp) {
  return (float)raw_temp / 16.0f;
}

static uint8_t onewire_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t inbyte = data[i];
    for (int j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01U;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8CU;
      }
      inbyte >>= 1;
    }
  }
  return crc;
}

static uint32_t ds18b20_conversion_time_ms(ds18b20_resolution_t resolution) {
  switch (resolution) {
  case DS18B20_RESOLUTION_9BIT:
    return 94;
  case DS18B20_RESOLUTION_10BIT:
    return 188;
  case DS18B20_RESOLUTION_11BIT:
    return 375;
  case DS18B20_RESOLUTION_12BIT:
  default:
    return 750;
  }
}

static uint8_t ds18b20_config_for_resolution(ds18b20_resolution_t resolution) {
  switch (resolution) {
  case DS18B20_RESOLUTION_9BIT:
    return DS18B20_CONFIG_RESERVED;
  case DS18B20_RESOLUTION_10BIT:
    return DS18B20_CONFIG_RESERVED | DS18B20_CONFIG_R0;
  case DS18B20_RESOLUTION_11BIT:
    return DS18B20_CONFIG_RESERVED | DS18B20_CONFIG_R1;
  case DS18B20_RESOLUTION_12BIT:
  default:
    return DS18B20_CONFIG_RESERVED | DS18B20_CONFIG_R1 | DS18B20_CONFIG_R0;
  }
}

static ds18b20_resolution_t ds18b20_resolution_from_config(uint8_t config) {
  switch (config & DS18B20_CONFIG_RES_MASK) {
  case 0:
    return DS18B20_RESOLUTION_9BIT;
  case DS18B20_CONFIG_R0:
    return DS18B20_RESOLUTION_10BIT;
  case DS18B20_CONFIG_R1:
    return DS18B20_RESOLUTION_11BIT;
  case DS18B20_CONFIG_R1 | DS18B20_CONFIG_R0:
  default:
    return DS18B20_RESOLUTION_12BIT;
  }
}

static bool ds18b20_lock(void) {
  return s_mutex != NULL && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE;
}

static void ds18b20_unlock(void) {
  xSemaphoreGive(s_mutex);
}

esp_err_t isolate_ds18b20_pin(void) {
  const bool locked = s_mutex != NULL && ds18b20_lock();
  if (locked) {
    s_state = DS18B20_STATE_IDLE;
    s_conversion_start_us = 0;
  }
  gpio_pullup_dis(DS18B20_PIN);
  gpio_pulldown_dis(DS18B20_PIN);
  esp_err_t ret = rtc_gpio_isolate(DS18B20_PIN);
  if (locked) {
    ds18b20_unlock();
  }
  return ret;
}

esp_err_t deisolate_ds18b20_pin(void) {
  const bool locked = s_mutex != NULL && ds18b20_lock();
  esp_err_t ret = gpio_hold_dis(DS18B20_PIN);
  esp_err_t deinit_ret = rtc_gpio_deinit(DS18B20_PIN);
  if (ret == ESP_OK) {
    ret = deinit_ret;
  }

  esp_err_t pull_ret = gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
  if (ret == ESP_OK) {
    ret = pull_ret;
  }
  if (locked) {
    ds18b20_unlock();
  }
  return ret;
}

esp_err_t drv_ds18b20_init(void) {
  if (s_mutex == NULL) {
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    if (s_mutex == NULL) {
      return ESP_ERR_NO_MEM;
    }
  }

  if (!ds18b20_lock()) {
    return ESP_FAIL;
  }

  if (g_ds18b20_initialized) {
    LOG_DEBUG("DS18B20 driver already initialized");
    ds18b20_unlock();
    return ESP_OK;
  }

  esp_err_t ret = gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  if (ret != ESP_OK) {
    LOG_ERRORF("Failed to configure DS18B20 GPIO: %s", esp_err_to_name(ret));
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);

  ret = onewire_reset();
  if (ret != ESP_OK) {
    LOG_WARN("DS18B20 not detected during init");
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  g_ds18b20_initialized = true;
  s_conversion_start_us = 0;
  s_state = DS18B20_STATE_IDLE;

  uint8_t scratchpad[DS18B20_SCRATCHPAD_BYTES] = {0};
  ret = ds18b20_read_scratchpad(scratchpad);
  if (ret == ESP_OK) {
    s_current_resolution =
        ds18b20_resolution_from_config(scratchpad[DS18B20_CONFIG_INDEX]);
  } else {
    LOG_WARNF("Failed to read DS18B20 resolution, using 12-bit timing: %s",
              esp_err_to_name(ret));
    s_current_resolution = DS18B20_RESOLUTION_12BIT;
  }

  LOG_DEBUG("DS18B20 initialized successfully");
  ds18b20_unlock();
  return ESP_OK;
}

esp_err_t drv_ds18b20_set_resolution(ds18b20_resolution_t resolution) {
  if (!g_ds18b20_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (resolution > DS18B20_RESOLUTION_12BIT) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!ds18b20_lock()) {
    return ESP_FAIL;
  }
  if (s_state == DS18B20_STATE_CONVERTING ||
      s_state == DS18B20_STATE_READY) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t scratchpad[DS18B20_SCRATCHPAD_BYTES] = {0};
  esp_err_t ret = ds18b20_read_scratchpad(scratchpad);
  if (ret != ESP_OK) {
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  uint8_t config = ds18b20_config_for_resolution(resolution);
  if ((scratchpad[DS18B20_CONFIG_INDEX] & DS18B20_CONFIG_RES_MASK) ==
      (config & DS18B20_CONFIG_RES_MASK)) {
    s_current_resolution = resolution;
    s_state = DS18B20_STATE_IDLE;
    ds18b20_unlock();
    return ESP_OK;
  }

  ret = ds18b20_write_scratchpad(scratchpad[DS18B20_TH_INDEX],
                                 scratchpad[DS18B20_TL_INDEX], config);
  if (ret != ESP_OK) {
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  ret = ds18b20_copy_scratchpad();
  if (ret != ESP_OK) {
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  s_current_resolution = resolution;
  s_state = DS18B20_STATE_IDLE;
  LOG_DEBUGF("DS18B20 resolution set to %d-bit", 9 + resolution);
  ds18b20_unlock();
  return ESP_OK;
}

esp_err_t drv_ds18b20_start_conversion(void) {
  if (!g_ds18b20_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!ds18b20_lock()) {
    return ESP_FAIL;
  }
  if (s_state == DS18B20_STATE_CONVERTING ||
      s_state == DS18B20_STATE_READY) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = onewire_reset();
  if (ret != ESP_OK) {
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  onewire_write_byte(DS18B20_CMD_SKIP_ROM);
  onewire_write_byte(DS18B20_CMD_CONVERT_T);
  set_gpio_strong_high();
  s_conversion_start_us = esp_timer_get_time();
  s_state = DS18B20_STATE_CONVERTING;
  LOG_INFO("DS18B20 conversion started");
  ds18b20_unlock();
  return ESP_OK;
}

ds18b20_state_t drv_ds18b20_get_state(void) {
  if (s_mutex == NULL || !ds18b20_lock()) {
    return DS18B20_STATE_ERROR;
  }
  ds18b20_state_t state = s_state;
  ds18b20_unlock();
  return state;
}

esp_err_t drv_ds18b20_get_conversion_timing(int64_t *elapsed_us,
                                             int64_t *remaining_us) {
  if (elapsed_us == NULL || remaining_us == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!g_ds18b20_initialized || !ds18b20_lock()) {
    return ESP_ERR_INVALID_STATE;
  }
  if ((s_state != DS18B20_STATE_CONVERTING &&
       s_state != DS18B20_STATE_READY) ||
      s_conversion_start_us <= 0) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }

  const int64_t now_us = esp_timer_get_time();
  const int64_t required_us =
      (int64_t)ds18b20_conversion_time_ms(s_current_resolution) * 1000LL;
  *elapsed_us = now_us > s_conversion_start_us
                    ? now_us - s_conversion_start_us
                    : 0;
  *remaining_us = required_us > *elapsed_us ? required_us - *elapsed_us : 0;
  if (*remaining_us == 0) {
    s_state = DS18B20_STATE_READY;
  }
  ds18b20_unlock();
  return ESP_OK;
}

esp_err_t drv_ds18b20_read_conversion_result(float *temperature) {
  if (!g_ds18b20_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (temperature == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!ds18b20_lock()) {
    return ESP_FAIL;
  }
  if ((s_state != DS18B20_STATE_CONVERTING &&
       s_state != DS18B20_STATE_READY) ||
      s_conversion_start_us <= 0) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }

  const int64_t required_us =
      (int64_t)ds18b20_conversion_time_ms(s_current_resolution) * 1000LL;
  const int64_t now_us = esp_timer_get_time();
  const int64_t elapsed_us = now_us > s_conversion_start_us
                                 ? now_us - s_conversion_start_us
                                 : 0;
  if (elapsed_us < required_us) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }
  s_state = DS18B20_STATE_READY;

  const int64_t scratchpad_read_start_us = esp_timer_get_time();
  esp_err_t last_err = ESP_FAIL;
  for (int attempt = 0; attempt <= DS18B20_MAX_READ_RETRIES; ++attempt) {
    uint8_t scratchpad[DS18B20_SCRATCHPAD_BYTES] = {0};
    esp_err_t ret = ds18b20_read_scratchpad(scratchpad);
    if (ret != ESP_OK) {
      last_err = ret;
      if (ret == ESP_ERR_INVALID_CRC &&
          attempt < DS18B20_MAX_READ_RETRIES) {
        vTaskDelay(pdMS_TO_TICKS(DS18B20_CRC_RETRY_DELAY_MS));
      }
      continue;
    }

    int16_t raw_temp =
        (int16_t)(((uint16_t)scratchpad[DS18B20_TEMP_MSB_INDEX] << 8) |
                  scratchpad[DS18B20_TEMP_LSB_INDEX]);
    if ((uint16_t)raw_temp == DS18B20_POWER_ON_DEFAULT_RAW) {
      LOG_WARN("DS18B20 rejected power-on default temperature: 85.000 C");
      last_err = ESP_ERR_INVALID_RESPONSE;
      continue;
    }

    float temp_c = ds18b20_raw_to_celsius(raw_temp);
    if (temp_c < DS18B20_TEMP_MIN_C || temp_c > DS18B20_TEMP_MAX_C) {
      LOG_WARNF("DS18B20 temperature out of range: raw=0x%04X temp=%.3f C",
                (uint16_t)raw_temp, temp_c);
      last_err = ESP_ERR_INVALID_RESPONSE;
      continue;
    }

    *temperature = temp_c;
    const int64_t scratchpad_read_us =
        esp_timer_get_time() - scratchpad_read_start_us;
    const int64_t total_elapsed_us = esp_timer_get_time() - s_conversion_start_us;
    LOG_INFOF("DS18B20 conversion elapsed: %lld ms",
              (long long)(total_elapsed_us / 1000LL));
    LOG_INFOF("DS18B20 scratchpad read time: %lld ms",
              (long long)((scratchpad_read_us + 999LL) / 1000LL));
    LOG_INFOF("DS18B20 temperature read: %.3f C", temp_c);
    s_conversion_start_us = 0;
    s_state = DS18B20_STATE_IDLE;
    ds18b20_unlock();
    return ESP_OK;
  }

  s_conversion_start_us = 0;
  s_state = DS18B20_STATE_ERROR;
  const int64_t scratchpad_read_us =
      esp_timer_get_time() - scratchpad_read_start_us;
  LOG_WARNF("DS18B20 scratchpad read failed after %lld ms: %s",
            (long long)((scratchpad_read_us + 999LL) / 1000LL),
            esp_err_to_name(last_err));
  ds18b20_unlock();
  return last_err;
}

esp_err_t drv_ds18b20_self_test(void) {
  if (!g_ds18b20_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  LOG_DEBUG("Starting DS18B20 self-test...");

  if (!ds18b20_lock()) {
    return ESP_FAIL;
  }
  if (s_state == DS18B20_STATE_CONVERTING ||
      s_state == DS18B20_STATE_READY) {
    ds18b20_unlock();
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = onewire_reset();
  if (ret != ESP_OK) {
    LOG_ERROR("Self-test failed: bus reset error");
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  uint8_t scratchpad[DS18B20_SCRATCHPAD_BYTES] = {0};
  ret = ds18b20_read_scratchpad(scratchpad);
  if (ret != ESP_OK) {
    LOG_ERROR("Self-test failed: cannot read DS18B20 scratchpad");
    s_state = DS18B20_STATE_ERROR;
    ds18b20_unlock();
    return ret;
  }

  LOG_DEBUGF("DS18B20 scratchpad temp_lsb=0x%02X temp_msb=0x%02X th=0x%02X "
             "tl=0x%02X cfg=0x%02X",
             scratchpad[DS18B20_TEMP_LSB_INDEX],
             scratchpad[DS18B20_TEMP_MSB_INDEX], scratchpad[DS18B20_TH_INDEX],
             scratchpad[DS18B20_TL_INDEX], scratchpad[DS18B20_CONFIG_INDEX]);
  s_state = DS18B20_STATE_IDLE;
  ds18b20_unlock();

  ret = drv_ds18b20_start_conversion();
  if (ret != ESP_OK) {
    LOG_ERROR("Self-test failed: cannot start temperature conversion");
    return ret;
  }

  int64_t elapsed_us = 0;
  int64_t remaining_us = 0;
  do {
    ret = drv_ds18b20_get_conversion_timing(&elapsed_us, &remaining_us);
    if (ret != ESP_OK) {
      return ret;
    }
    if (remaining_us > 0) {
      const uint32_t remaining_ms =
          (uint32_t)((remaining_us + 999LL) / 1000LL);
      vTaskDelay(pdMS_TO_TICKS(remaining_ms));
    }
  } while (remaining_us > 0);

  float temperature = 0.0f;
  ret = drv_ds18b20_read_conversion_result(&temperature);
  if (ret != ESP_OK) {
    LOG_ERROR("Self-test failed: cannot read DS18B20 temperature");
    return ret;
  }

  if (temperature < DS18B20_TEMP_MIN_C || temperature > DS18B20_TEMP_MAX_C) {
    LOG_WARNF("DS18B20 temperature outside absolute range: %.3f C",
              temperature);
  } else {
    LOG_DEBUGF("DS18B20 temperature reading valid: %.3f C", temperature);
  }

  LOG_DEBUG("DS18B20 self-test passed");
  return ESP_OK;
}
