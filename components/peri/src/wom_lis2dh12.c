#include "wom_lis2dh12.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "logger.h"
#include "drv_lis2dh12.h"

static const char *TAG = "WOM_LIS2DH12";

static QueueHandle_t gpio_evt_queue = NULL;

// Default WoM configuration to be restored after FFT capture
static lis2dh12_wom_cfg_t s_default_wom_cfg = {
    // Start with conservative/debounced values to avoid chatter on a
    // stationary desk. These can be tuned further by the application.
    .threshold_mg_int1 = 3000,  // ~3g for vibration/shock detection
    .threshold_mg_int2 = 4000,  // ~4g posture deviation (large movement)
    .duration_int1 = 3,         // ~60ms at 50Hz to filter brief spikes
    .duration_int2 = 20,        // 400ms debounce for posture (50Hz)
};

// ISR handler
// ISR now quickly clears the sensor latch and disables further
// interrupts for that pin until the main task has processed the event.
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPri = pdFALSE;

    // clear the source register so the latch is released immediately
    if (gpio_num == LIS2DH12_PIN_NUM_INT1) {
        uint8_t dummy;
        drv_lis2dh12_read_int1_source(&dummy);
    } else if (gpio_num == LIS2DH12_PIN_NUM_INT2) {
        uint8_t dummy;
        drv_lis2dh12_read_int2_source(&dummy);
    }

    // enqueue the pin for processing; drop if queue is full
    if (gpio_evt_queue) {
        (void) xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPri);
        if (xHigherPri) portYIELD_FROM_ISR();
    }

    // disable further interrupts on this line until task handles it
    gpio_intr_disable(gpio_num);
}

// Main task
static void wom_listener_task(void* arg) {
    uint32_t io_num;
    ESP_LOGI(TAG, "WoM listener task started. Waiting for WoM event...");

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // throttle burst of events (helps when line is latched)
            vTaskDelay(pdMS_TO_TICKS(50));
            // re‑enable GPIO interrupt now that we are about to handle it
            gpio_intr_enable(io_num);
            // --- Stage 1: WoM Interrupt Received ---
            if (io_num == LIS2DH12_PIN_NUM_INT1) {
                LOG_WARNF(TAG, "WoM Event: Vibration Anomaly (INT1, %dmg) triggered.", s_default_wom_cfg.threshold_mg_int1);
            } else if (io_num == LIS2DH12_PIN_NUM_INT2) {
                LOG_ERRORF(TAG, "WoM Event: Posture Deviation (INT2, %dmg) triggered.", s_default_wom_cfg.threshold_mg_int2);
            }
            // dump a raw sample to see HPF output
            {
                lis2dh12_raw_data_t sample;
                if (drv_lis2dh12_get_raw_data(&sample) == ESP_OK) {
                    // compute mg value using current full-scale
                    int fs_index = drv_lis2dh12_get_current_fs();
                    int sensitivity_mg = 1;
                    switch (fs_index) {
                        case LIS2DH12_FS_2G:  sensitivity_mg = 1; break;
                        case LIS2DH12_FS_4G:  sensitivity_mg = 2; break;
                        case LIS2DH12_FS_8G:  sensitivity_mg = 4; break;
                        case LIS2DH12_FS_16G: sensitivity_mg = 12; break;
                    }
                    ESP_LOGI(TAG, "raw(accel): X=%d (%dmg) Y=%d (%dmg) Z=%d (%dmg) FS=%dG",
                             sample.x, sample.x * sensitivity_mg,
                             sample.y, sample.y * sensitivity_mg,
                             sample.z, sample.z * sensitivity_mg,
                             (1 << fs_index) * 2);
                }
            }
            // --- Stage 1.5: Read interrupt source to clear latched interrupt ---
            // This is CRITICAL: With LIR (Latch Interrupt Request) enabled,
            // the interrupt remains asserted until the source register is read
            uint8_t int_src = 0;
            if (io_num == LIS2DH12_PIN_NUM_INT1) {
                drv_lis2dh12_read_int1_source(&int_src);
                ESP_LOGI(TAG, "INT1_SRC=0x%02X (X=%d Y=%d Z=%d)",
                         int_src, (int_src>>0)&1, (int_src>>2)&1, (int_src>>4)&1);
            } else if (io_num == LIS2DH12_PIN_NUM_INT2) {
                drv_lis2dh12_read_int2_source(&int_src);
                ESP_LOGI(TAG, "INT2_SRC=0x%02X (X=%d Y=%d Z=%d)",
                         int_src, (int_src>>0)&1, (int_src>>2)&1, (int_src>>4)&1);
            }

            ESP_LOGI(TAG, "WoM event handled. Continuing to listen...");
        }
    }
}

esp_err_t start_wom_lis2dh12_listener() {
    // Create a queue to handle gpio events from isr
    
    esp_err_t ret = drv_lis2dh12_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LIS2DH12: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue.");
        return ESP_ERR_NO_MEM;
    }

    // Create the listener task
    // bump stack size because logging + GPIO handling can be stack-heavy
    BaseType_t task_created = xTaskCreate(wom_listener_task, "wom_listener_task", 4096, NULL, 10, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create WoM listener task.");
        vQueueDelete(gpio_evt_queue);
        return ESP_FAIL;
    }
    
    // Configure GPIOs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LIS2DH12_PIN_NUM_INT1) | (1ULL << LIS2DH12_PIN_NUM_INT2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install gpio isr service
    ret = gpio_install_isr_service(0); // ESP_INTR_FLAG_DEFAULT = 0
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO isr service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handlers for each pin
    ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT1, gpio_isr_handler, (void*) LIS2DH12_PIN_NUM_INT1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR handler add for INT1 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT2, gpio_isr_handler, (void*) LIS2DH12_PIN_NUM_INT2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR handler add for INT2 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable WoM on the sensor
    ret = drv_lis2dh12_enable_wom(&s_default_wom_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable WoM: %s", esp_err_to_name(ret));
        // Cleanup ISR handlers
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT2);
        // The task and queue are not cleaned up here to maintain consistency with existing error handling.
        return ret;
    }
    // dump configuration registers for debugging
    {
        uint8_t reg2, reg3, reg5, thr1, thr2;
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG2, &reg2);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG3, &reg3);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG5, &reg5);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT1_THS, &thr1);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT2_THS, &thr2);
        ESP_LOGI(TAG, "post‑WoM regs: CTRL2=0x%02X CTRL3=0x%02X CTRL5=0x%02X THS1=0x%02X THS2=0x%02X",
                 reg2, reg3, reg5, thr1, thr2);
    }

    ESP_LOGI(TAG, "WoM listener initialized and enabled successfully.");
    return ESP_OK;
}

// Check GPIO state after waking up from light sleep
// Since edge interrupts might be missed during sleep, we check the level manually.
void wom_lis2dh12_on_wakeup(void) {
    if (gpio_get_level(LIS2DH12_PIN_NUM_INT1)) {
        uint32_t io_num = LIS2DH12_PIN_NUM_INT1;
        xQueueSend(gpio_evt_queue, &io_num, 0);
    }
    if (gpio_get_level(LIS2DH12_PIN_NUM_INT2)) {
        uint32_t io_num = LIS2DH12_PIN_NUM_INT2;
        xQueueSend(gpio_evt_queue, &io_num, 0);
    }
}
