#include "wom_lis2dh12.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "drv_lis2dh12.h"

static const char *TAG = "WOM_LIS2DH12";

static QueueHandle_t gpio_evt_queue = NULL;

// Default WoM configuration to be restored after FFT capture
static const lis2dh12_wom_cfg_t s_default_wom_cfg = {
    .threshold_mg_int1 = 100, // 100mg threshold
    .threshold_mg_int2 = 200, // 200mg threshold
    .duration_int1 = 0,
    .duration_int2 = 0,
};

// ISR handler
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Callback for handling FIFO data packets
static void fft_data_handler(const lis2dh12_raw_data_t *data, size_t count) {
    ESP_LOGI(TAG, "FFT Capture: Received %d samples.", count);
    // In a real application, this data would be sent to an FFT processing task.
}

// Main task
static void wom_listener_task(void* arg) {
    uint32_t io_num;
    ESP_LOGI(TAG, "WoM listener task started. Waiting for WoM event...");

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // --- Stage 1: WoM Interrupt Received ---
            if (io_num == LIS2DH12_PIN_NUM_INT1) {
                ESP_LOGW(TAG, "WoM Event: Low-level interrupt (INT1) triggered. Starting detailed capture.");
            } else if (io_num == LIS2DH12_PIN_NUM_INT2) {
                ESP_LOGE(TAG, "WoM Event: High-level alarm (INT2) triggered. Starting detailed capture.");
            }

            // --- Stage 2: Dynamic Re-configuration for FFT Capture ---
            ESP_LOGI(TAG, "Disabling WoM mode...");
            ESP_ERROR_CHECK(drv_lis2dh12_disable_wom());

            ESP_LOGI(TAG, "Setting high-frequency ODR (400Hz) for FFT...");
            ESP_ERROR_CHECK(drv_lis2dh12_set_config(LIS2DH12_FS_2G, 400.0f));
            
            ESP_LOGI(TAG, "Starting FIFO capture...");
            ESP_ERROR_CHECK(drv_lis2dh12_start_fifo_capture(10, fft_data_handler));

            // Let the capture run for a while
            const int capture_duration_ms = 5000;
            ESP_LOGI(TAG, "Capturing data for %dms...", capture_duration_ms);
            vTaskDelay(pdMS_TO_TICKS(capture_duration_ms));

            ESP_LOGI(TAG, "Stopping FIFO capture...");
            ESP_ERROR_CHECK(drv_lis2dh12_stop_fifo_capture());

            // --- Stage 3: Restore WoM Mode ---
            ESP_LOGI(TAG, "Re-enabling WoM mode...");
            ESP_ERROR_CHECK(drv_lis2dh12_enable_wom(&s_default_wom_cfg));

            // Re-attach the WoM ISR handler for INT1, as the FIFO capture took ownership of it
            gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT1, gpio_isr_handler, (void*) LIS2DH12_PIN_NUM_INT1);
            
            ESP_LOGI(TAG, "Process complete. Returning to WoM listening mode.");
        }
    }
}

esp_err_t start_wom_lis2dh12_listener(void) {
    // Create a queue to handle gpio events from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue.");
        return ESP_ERR_NO_MEM;
    }

    // Create the listener task
    BaseType_t task_created = xTaskCreate(wom_listener_task, "wom_listener_task", 2048, NULL, 10, NULL);
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
    esp_err_t ret = gpio_config(&io_conf);
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

    ESP_LOGI(TAG, "WoM listener initialized successfully.");
    return ESP_OK;
}
