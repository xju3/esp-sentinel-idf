#include "wom_lis2dh12.h"
#include "drv_lis2dh12.h"
#include "logger.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"

static QueueHandle_t gpio_evt_queue = NULL;
#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

// ---------------------------------------------------------------------------
// Default WoM thresholds
//
// FS=±16g, normal mode → 186 mg/LSB (datasheet Table 59/68)
// INT_THS max = 127 × 186 = 23622 mg (~23g)
//
// INT1 — shock / vibration / motor e-stop
//   Target: events ≥ ~2g (2000 mg)
//   Register value: 2000 / 186 = 10 LSb → actual threshold 1860 mg
//   Duration: 2 × 20 ms = 40 ms
//
// INT2 — posture deviation / base fracture
//   HPF with autoreset removes gravity; only sustained shifts appear.
//   Target: shifts ≥ ~1g (1000 mg) from current HPF baseline
//   Register value: 1000 / 186 = 5 LSb → actual threshold 930 mg
//   Duration: 5 × 20 ms = 100 ms
//
// Tune on real hardware — raise thresholds until false alarms stop
// while confirmed events still trigger reliably.
// ---------------------------------------------------------------------------
static lis2dh12_wom_cfg_t s_default_wom_cfg = {
    .threshold_mg_int1 = 188, // vibration/shock
    .duration_int1 = 2,
    .threshold_mg_int2 = 2000, // posture/orientation
    .duration_int2 = 5,
};

static void enable_lis2dh12_gpoi_wakeup()
{
    // Configure LIS2DH12 Interrupt pins as wakeup sources
    // LIS2DH12 interrupts are Active High (configured in drv_lis2dh12.c)
    gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT1, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT2, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}
// ---------------------------------------------------------------------------
// ISR handler
//
// IMPORTANT: drv_lis2dh12_read_int*_source() performs SPI I/O, which must
// NOT be called from an ISR. Interrupt source clearing is moved to the task.
// The ISR only enqueues the pin number and disables the GPIO until the task
// has processed the event, preventing burst re-triggering.
// ---------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPri = pdFALSE;

    if (gpio_evt_queue)
    {
        (void)xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPri);
        if (xHigherPri)
            portYIELD_FROM_ISR();
    }

    // Disable this GPIO line until the listener task has handled the event
    gpio_intr_disable(gpio_num);
}

// ---------------------------------------------------------------------------
// Listener task
// ---------------------------------------------------------------------------
static void wom_listener_task(void *arg)
{
    uint32_t io_num;
    LOG_DEBUG("WoM listener task started.");

    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {

            uint8_t int_src = 0;
            vTaskDelay(pdMS_TO_TICKS(5));
            if (io_num == LIS2DH12_PIN_NUM_INT1)
            {
                drv_lis2dh12_read_int1_source(&int_src);
                // 如果只是为了中断而不需要加速度数据，可以将这部分LOG信息注释掉。
                LOG_WARNF("WoM INT1: shock/vibration (thr=%d mg) INT1_SRC=0x%02X",
                          s_default_wom_cfg.threshold_mg_int1, int_src);
            }
            else if (io_num == LIS2DH12_PIN_NUM_INT2)
           {
                drv_lis2dh12_read_int2_source(&int_src);
                LOG_ERRORF("WoM INT2: posture deviation (thr=%d mg) INT2_SRC=0x%02X",
                           s_default_wom_cfg.threshold_mg_int2, int_src);
            }
            gpio_intr_enable(io_num);
            // TODO: trigger health-check or wakeup sequence here
        }
    }
}

// ---------------------------------------------------------------------------
// start_wom_lis2dh12_listener
// ---------------------------------------------------------------------------
esp_err_t start_wom_lis2dh12_listener(void)
{
    gpio_evt_queue = xQueueCreateWithCaps(10, sizeof(uint32_t), TASK_MEM_CAPS);
    if (!gpio_evt_queue)
    {
        LOG_ERRORF("Failed to create GPIO event queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_created =
        xTaskCreateWithCaps(wom_listener_task, "wom_listener", 4096, NULL, 10, NULL, TASK_MEM_CAPS);
    if (task_created != pdPASS)
    {
        LOG_ERRORF("Failed to create WoM listener task");
        vQueueDeleteWithCaps(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ESP_FAIL;
    }

    // Configure GPIO inputs for INT1 and INT2
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIS2DH12_PIN_NUM_INT1) | (1ULL << LIS2DH12_PIN_NUM_INT2),
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable wakeup after configuration to ensure pin state is defined
    enable_lis2dh12_gpoi_wakeup();

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        LOG_ERRORF("GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT1, gpio_isr_handler, (void *)LIS2DH12_PIN_NUM_INT1);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("ISR handler add INT1 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT2, gpio_isr_handler, (void *)LIS2DH12_PIN_NUM_INT2);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("ISR handler add INT2 failed: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
        return ret;
    }

    // Arm the sensor
    ret = drv_lis2dh12_enable_wom(&s_default_wom_cfg);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable WoM: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT2);
        return ret;
    }

    // Dump final register state for verification
    {
        uint8_t r1, r2, r3, r4, r5, r6, thr1, thr2, dur1, dur2, cfg1, cfg2;
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG1, &r1);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG2, &r2);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG3, &r3);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG4, &r4);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG5, &r5);
        drv_lis2dh12_read_register(LIS2DH12_REG_CTRL_REG6, &r6);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT1_THS, &thr1);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT2_THS, &thr2);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT1_DURATION, &dur1);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT2_DURATION, &dur2);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT1_CFG, &cfg1);
        drv_lis2dh12_read_register(LIS2DH12_REG_INT2_CFG, &cfg2);
        LOG_DEBUG("WoM register dump:");
        LOG_DEBUGF("  CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL4=0x%02X CTRL5=0x%02X CTRL6=0x%02X",
                   r1, r2, r3, r4, r5, r6);
        LOG_DEBUGF("  INT1: CFG=0x%02X THS=0x%02X(%umg) DUR=0x%02X(%ums)",
                   cfg1, thr1, thr1 * 186u, dur1, dur1 * 20u);
        LOG_DEBUGF("  INT2: CFG=0x%02X THS=0x%02X (6D mode, THS unused) DUR=0x%02X(%ums)",
                   cfg2, thr2, dur2, dur2 * 20u);
    }

    LOG_DEBUG("WoM listener ready.");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// wom_lis2dh12_on_wakeup
// Call this after returning from light sleep to catch edges missed during sleep.
// ---------------------------------------------------------------------------
void wom_lis2dh12_on_wakeup(void)
{
    if (!gpio_evt_queue)
        return;
    if (gpio_get_level(LIS2DH12_PIN_NUM_INT1))
    {
        uint32_t io_num = LIS2DH12_PIN_NUM_INT1;
        xQueueSend(gpio_evt_queue, &io_num, 0);
    }
    if (gpio_get_level(LIS2DH12_PIN_NUM_INT2))
    {
        uint32_t io_num = LIS2DH12_PIN_NUM_INT2;
        xQueueSend(gpio_evt_queue, &io_num, 0);
    }
}
