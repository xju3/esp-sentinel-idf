#include "drv_tmb12a05.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "logger.h"
#include <string.h>

// 私有函数声明
static void beep_task(void *pvParameters);
static void beep_timer_callback(TimerHandle_t xTimer);
static void update_buzzer_state(bool state);
static void notify_state_change(tmb12a05_mode_t new_mode);

// Task stack: keep generous headroom (ESP-IDF logging can be stack-hungry).
#define TMB12A05_TASK_STACK 4096

// 驱动状态结构
typedef struct {
    bool initialized;
    tmb12a05_mode_t current_mode;
    tmb12a05_config_t config;
    tmb12a05_state_cb_t state_callback;
    TaskHandle_t beep_task_handle;
    TimerHandle_t beep_timer;
    uint8_t remaining_beeps;
    uint64_t last_beep_time;
    gpio_num_t gpio_num;
    bool restore_config_pending;
    tmb12a05_config_t restore_config;
} drv_tmb12a05_state_t;

// 全局驱动状态
static drv_tmb12a05_state_t s_state = {
    .initialized = false,
    .current_mode = TMB12A05_MODE_OFF,
    .config = {
        .beep_duration_ms = 100,
        .beep_interval_ms = 100,
        .beep_count = 1,
        .active_high =
#ifdef CONFIG_PERI_TMB12A05_ACTIVE_HIGH
            CONFIG_PERI_TMB12A05_ACTIVE_HIGH
#else
            true
#endif
    },
    .state_callback = NULL,
    .beep_task_handle = NULL,
    .beep_timer = NULL,
    .remaining_beeps = 0,
    .last_beep_time = 0,
    .gpio_num = TMB12A05_PIN,
    .restore_config_pending = false,
    .restore_config = {
        .beep_duration_ms = 100,
        .beep_interval_ms = 100,
        .beep_count = 1,
        .active_high =
#ifdef CONFIG_PERI_TMB12A05_ACTIVE_HIGH
            CONFIG_PERI_TMB12A05_ACTIVE_HIGH
#else
            true
#endif
    }
};

// 蜂鸣任务函数
static void beep_task(void *pvParameters) {
    LOG_DEBUG("Buzzer beep task started");
    
    while (1) {
        if (s_state.remaining_beeps > 0) {
            // 执行蜂鸣
            update_buzzer_state(true);  // 打开蜂鸣器
            vTaskDelay(pdMS_TO_TICKS(s_state.config.beep_duration_ms));
            update_buzzer_state(false); // 关闭蜂鸣器
            
            s_state.remaining_beeps--;
            s_state.last_beep_time = esp_timer_get_time();
            
            if (s_state.remaining_beeps > 0) {
                // 等待间隔时间
                vTaskDelay(pdMS_TO_TICKS(s_state.config.beep_interval_ms));
            } else {
                // 蜂鸣完成，更新状态
                if (s_state.restore_config_pending) {
                    memcpy(&s_state.config, &s_state.restore_config, sizeof(tmb12a05_config_t));
                    s_state.restore_config_pending = false;
                }
                s_state.current_mode = TMB12A05_MODE_OFF;
                notify_state_change(TMB12A05_MODE_OFF);
            }
        } else if (s_state.current_mode == TMB12A05_MODE_BEEP_CONTINUOUS) {
            // 连续蜂鸣模式
            update_buzzer_state(true);
            vTaskDelay(pdMS_TO_TICKS(s_state.config.beep_duration_ms));
            update_buzzer_state(false);
            vTaskDelay(pdMS_TO_TICKS(s_state.config.beep_interval_ms));
        } else {
            // 无蜂鸣任务，等待信号
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 蜂鸣定时器回调（用于简单定时蜂鸣）
static void beep_timer_callback(TimerHandle_t xTimer) {
    if (s_state.current_mode == TMB12A05_MODE_ON) {
        update_buzzer_state(false);
        s_state.current_mode = TMB12A05_MODE_OFF;
        notify_state_change(TMB12A05_MODE_OFF);
        LOG_DEBUG("Buzzer timer expired, turning off");
    }
}

// 更新蜂鸣器硬件状态
static void update_buzzer_state(bool state) {
    int level = state ? 1 : 0;
    
    // 根据active_high配置调整电平
    if (!s_state.config.active_high) {
        level = !level;
    }
    
    gpio_set_level(s_state.gpio_num, level);
}

// 通知状态变化
static void notify_state_change(tmb12a05_mode_t new_mode) {
    if (s_state.state_callback != NULL) {
        s_state.state_callback(new_mode);
    }
}

// 公共函数实现

esp_err_t drv_tmb12a05_init(void) {
    if (s_state.initialized) {
        LOG_DEBUG("Buzzer already initialized");
        return ESP_OK;
    }
    
    LOG_DEBUG("Initializing TMB12A05 buzzer...");

    s_state.gpio_num = TMB12A05_PIN;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(s_state.gpio_num)) {
        LOG_ERRORF("Invalid buzzer GPIO (not output-capable): %d", (int)s_state.gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    LOG_DEBUGF("Buzzer GPIO=%d, active_%s", (int)s_state.gpio_num, s_state.config.active_high ? "high" : "low");
    
    // 配置GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_state.gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        LOG_ERRORF("Failed to configure buzzer GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Best-effort: stronger drive in case the buzzer input needs it (e.g., via transistor base).
    (void)gpio_set_drive_capability(s_state.gpio_num, GPIO_DRIVE_CAP_3);
    
    // 初始状态：关闭蜂鸣器
    update_buzzer_state(false);
    
    // 创建蜂鸣任务
    BaseType_t task_ret = xTaskCreate(
        beep_task,
        "buzzer_task",
        TMB12A05_TASK_STACK,
        NULL,
        5,  // 中等优先级
        &s_state.beep_task_handle
    );
    
    if (task_ret != pdPASS) {
        LOG_ERROR("Failed to create buzzer task");
        gpio_reset_pin(s_state.gpio_num);
        return ESP_FAIL;
    }
    
    // 创建蜂鸣定时器
    s_state.beep_timer = xTimerCreate(
        "buzzer_timer",
        pdMS_TO_TICKS(100),  // 默认100ms，实际使用时会被重新配置
        pdFALSE,             // 单次定时器
        NULL,
        beep_timer_callback
    );
    
    if (s_state.beep_timer == NULL) {
        LOG_ERROR("Failed to create buzzer timer");
        vTaskDelete(s_state.beep_task_handle);
        s_state.beep_task_handle = NULL;
        gpio_reset_pin(s_state.gpio_num);
        return ESP_FAIL;
    }
    
    s_state.initialized = true;
    s_state.current_mode = TMB12A05_MODE_OFF;
    
    LOG_DEBUG("TMB12A05 buzzer initialized successfully");
    return ESP_OK;
}

esp_err_t drv_tmb12a05_deinit(void) {
    if (!s_state.initialized) {
        return ESP_OK;
    }
    
    LOG_DEBUG("Deinitializing TMB12A05 buzzer...");
    
    // 停止所有蜂鸣
    drv_tmb12a05_stop();
    
    // 删除定时器
    if (s_state.beep_timer != NULL) {
        xTimerDelete(s_state.beep_timer, portMAX_DELAY);
        s_state.beep_timer = NULL;
    }
    
    // 删除任务
    if (s_state.beep_task_handle != NULL) {
        vTaskDelete(s_state.beep_task_handle);
        s_state.beep_task_handle = NULL;
    }
    
    // 重置GPIO
    gpio_reset_pin(s_state.gpio_num);
    
    s_state.initialized = false;
    s_state.current_mode = TMB12A05_MODE_OFF;
    s_state.state_callback = NULL;
    s_state.restore_config_pending = false;
    
    LOG_DEBUG("TMB12A05 buzzer deinitialized successfully");
    return ESP_OK;
}

esp_err_t drv_tmb12a05_set_mode(tmb12a05_mode_t mode) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (mode >= TMB12A05_MODE_BEEP_CONTINUOUS + 1) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 如果模式相同，直接返回
    if (s_state.current_mode == mode) {
        return ESP_OK;
    }
    
    // LOG_DEBUGF("Setting buzzer mode: %d", mode);
    
    // 停止当前模式
    update_buzzer_state(false);
    
    switch (mode) {
        case TMB12A05_MODE_OFF:
            // 已经关闭
            s_state.remaining_beeps = 0;
            break;
            
        case TMB12A05_MODE_ON:
            s_state.remaining_beeps = 0;
            update_buzzer_state(true);
            break;
            
        case TMB12A05_MODE_BEEP_SINGLE:
            if (s_state.remaining_beeps == 0) s_state.remaining_beeps = 1;
            break;
            
        case TMB12A05_MODE_BEEP_DOUBLE:
            if (s_state.remaining_beeps == 0) s_state.remaining_beeps = 2;
            break;
            
        case TMB12A05_MODE_BEEP_TRIPLE:
            if (s_state.remaining_beeps == 0) s_state.remaining_beeps = 3;
            break;
            
        case TMB12A05_MODE_BEEP_CONTINUOUS:
            // 连续模式由任务处理
            s_state.remaining_beeps = 0;
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    s_state.current_mode = mode;
    notify_state_change(mode);
    
    return ESP_OK;
}

esp_err_t drv_tmb12a05_set_config(const tmb12a05_config_t *config) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 验证参数
    if (config->beep_duration_ms == 0 || config->beep_duration_ms > 5000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->beep_interval_ms > 10000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->beep_count > 10) {
        return ESP_ERR_INVALID_ARG;
    }

    // If a temporary beep is ongoing, update the restore target so that when the
    // temporary beep finishes it will restore to the user's new config.
    if (s_state.restore_config_pending) {
        memcpy(&s_state.restore_config, config, sizeof(tmb12a05_config_t));
        LOG_DEBUGF("Buzzer restore-config updated while beeping: duration=%dms, interval=%dms, count=%d, active_high=%d",
                   config->beep_duration_ms, config->beep_interval_ms,
                   config->beep_count, config->active_high);
        return ESP_OK;
    }

    memcpy(&s_state.config, config, sizeof(tmb12a05_config_t));
    
    LOG_DEBUGF("Buzzer config updated: duration=%dms, interval=%dms, count=%d, active_high=%d",
               config->beep_duration_ms, config->beep_interval_ms, 
               config->beep_count, config->active_high);
    
    return ESP_OK;
}

esp_err_t drv_tmb12a05_get_config(tmb12a05_config_t *config) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(config, &s_state.config, sizeof(tmb12a05_config_t));
    return ESP_OK;
}

tmb12a05_mode_t drv_tmb12a05_get_current_mode(void) {
    return s_state.current_mode;
}

esp_err_t drv_tmb12a05_beep_once(uint16_t duration_ms) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (duration_ms == 0 || duration_ms > 5000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUGF("Beeping once for %d ms", duration_ms);

    if (!s_state.restore_config_pending) {
        memcpy(&s_state.restore_config, &s_state.config, sizeof(tmb12a05_config_t));
        s_state.restore_config_pending = true;
    }

    s_state.config.beep_duration_ms = duration_ms;
    s_state.remaining_beeps = 1;
    return drv_tmb12a05_set_mode(TMB12A05_MODE_BEEP_SINGLE);
}

esp_err_t drv_tmb12a05_beep_multiple(uint8_t count, uint16_t duration_ms, uint16_t interval_ms) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (count == 0 || count > 10) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (duration_ms == 0 || duration_ms > 5000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (interval_ms > 10000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUGF("Beeping %d times (duration=%dms, interval=%dms)", count, duration_ms, interval_ms);

    if (!s_state.restore_config_pending) {
        memcpy(&s_state.restore_config, &s_state.config, sizeof(tmb12a05_config_t));
        s_state.restore_config_pending = true;
    }

    s_state.config.beep_duration_ms = duration_ms;
    s_state.config.beep_interval_ms = interval_ms;
    s_state.config.beep_count = count;
    s_state.remaining_beeps = count;

    // 根据次数选择模式（remaining_beeps 已设置，set_mode 不应覆盖）
    tmb12a05_mode_t mode;
    switch (count) {
        case 1:
            mode = TMB12A05_MODE_BEEP_SINGLE;
            break;
        case 2:
            mode = TMB12A05_MODE_BEEP_DOUBLE;
            break;
        case 3:
            mode = TMB12A05_MODE_BEEP_TRIPLE;
            break;
        default:
            mode = TMB12A05_MODE_BEEP_SINGLE; // 使用单次模式，但remaining_beeps已设置
            break;
    }

    return drv_tmb12a05_set_mode(mode);
}

esp_err_t drv_tmb12a05_start_continuous(uint16_t period_ms) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (period_ms < 50 || period_ms > 5000) {
        return ESP_ERR_INVALID_ARG;
    }
    
    LOG_DEBUGF("Starting continuous beeping with period %d ms", period_ms);
    
    // 设置连续蜂鸣参数
    s_state.config.beep_duration_ms = period_ms / 2;
    s_state.config.beep_interval_ms = period_ms / 2;
    
    // 启动连续模式
    return drv_tmb12a05_set_mode(TMB12A05_MODE_BEEP_CONTINUOUS);
}

esp_err_t drv_tmb12a05_stop(void) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("Stopping buzzer");
    
    // 停止蜂鸣
    update_buzzer_state(false);
    
    // 重置状态
    s_state.remaining_beeps = 0;

    // If we were using a temporary config for a one-shot beep, restore it now.
    if (s_state.restore_config_pending) {
        memcpy(&s_state.config, &s_state.restore_config, sizeof(tmb12a05_config_t));
        s_state.restore_config_pending = false;
    }
    
    // 如果当前是连续模式，需要停止任务中的循环
    if (s_state.current_mode == TMB12A05_MODE_BEEP_CONTINUOUS) {
        s_state.current_mode = TMB12A05_MODE_OFF;
        notify_state_change(TMB12A05_MODE_OFF);
    }
    
    // 停止定时器
    if (s_state.beep_timer != NULL) {
        xTimerStop(s_state.beep_timer, portMAX_DELAY);
    }
    
    return ESP_OK;
}

esp_err_t drv_tmb12a05_register_state_callback(tmb12a05_state_cb_t callback) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_state.state_callback = callback;
    LOG_DEBUG("Buzzer state callback registered");
    
    return ESP_OK;
}

esp_err_t drv_tmb12a05_self_test(void) {
    if (!s_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    LOG_DEBUG("Starting TMB12A05 buzzer self-test...");
    
    // 1. 测试单次蜂鸣
    LOG_DEBUG("Test 1: Single beep (100ms)");
    esp_err_t ret = drv_tmb12a05_beep_once(100);
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at single beep: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 2. 测试双次蜂鸣
    LOG_DEBUG("Test 2: Double beep");
    ret = drv_tmb12a05_beep_multiple(2, 80, 100);
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at double beep: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 3. 测试常开模式
    LOG_DEBUG("Test 3: Continuous ON for 300ms");
    ret = drv_tmb12a05_set_mode(TMB12A05_MODE_ON);
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at ON mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // 4. 测试停止
    LOG_DEBUG("Test 4: Stop");
    ret = drv_tmb12a05_stop();
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at stop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 5. 测试连续蜂鸣模式
    LOG_DEBUG("Test 5: Continuous beeping for 1 second");
    ret = drv_tmb12a05_start_continuous(200); // 200ms周期
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at continuous mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 6. 最终停止
    LOG_DEBUG("Test 6: Final stop");
    ret = drv_tmb12a05_stop();
    if (ret != ESP_OK) {
        LOG_ERRORF("Self-test failed at final stop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    LOG_DEBUG("TMB12A05 buzzer self-test passed successfully");
    return ESP_OK;
}
