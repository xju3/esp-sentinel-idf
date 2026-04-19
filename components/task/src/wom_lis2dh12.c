#include "wom_lis2dh12.h"
#include "drv_lis2dh12.h"
#include "logger.h"
#include "task_state_machine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#define TASK_MEM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

static TaskHandle_t s_wom_task = NULL;
static volatile uint32_t s_pending_events = 0;
static int s_int1_idle_level = -1;
static int s_int2_idle_level = -1;
static gpio_int_type_t s_int1_intr_type = GPIO_INTR_DISABLE;
static gpio_int_type_t s_int2_intr_type = GPIO_INTR_DISABLE;
static uint64_t s_int1_suppress_until_us = 0;
static bool s_listener_started = false;
static bool s_wom_armed = false;

// Bitmask for pending GPIO events (set in ISR, consumed in task)
#define WOM_EVT_INT1 (1u << 0)
#define WOM_EVT_INT2 (1u << 1)
#define WOM_INT1_COALESCE_MS 30u
#define WOM_SUPPRESS_INT1_AFTER_INT2_US 1000000u

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
    .fs = LIS2DH12_FS_2G,
    .threshold_mg_int1 = 600, // vibration/shock
    .duration_int1 = 3,        // 3 × 20 ms = 60 ms
    .threshold_mg_int2 = 1200, // posture/orientation
    .duration_int2 = 6,
};

static uint16_t lis2dh12_int_ths_step_mg(lis2dh12_fs_t fs)
{
    switch (fs)
    {
    case LIS2DH12_FS_2G:
        return 16u;
    case LIS2DH12_FS_4G:
        return 32u;
    case LIS2DH12_FS_8G:
        return 62u;
    case LIS2DH12_FS_16G:
    default:
        return 186u;
    }
}

static gpio_int_type_t wom_gpio_wakeup_type_for_idle(int idle_level)
{
    return (idle_level == 0) ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL;
}

static void wom_int1_clear_and_rearm(bool suppress_log)
{
    const uint32_t io_num = LIS2DH12_PIN_NUM_INT1;
    uint8_t int_src = 0;
    esp_err_t ret = drv_lis2dh12_read_int1_source(&int_src);
    if (ret != ESP_OK)
    {
        if (!suppress_log)
            LOG_ERRORF("Failed to read INT1_SRC: %s", esp_err_to_name(ret));
    }
    else if (!suppress_log)
    {
        if (int_src & 0x40) // IA bit set
        {
            // 如果只是为了中断而不需要加速度数据，可以将这部分LOG信息注释掉。
            LOG_WARNF("WoM INT1: shock/vibration (thr=%d mg) INT1_SRC=0x%02X", s_default_wom_cfg.threshold_mg_int1, int_src);
            // wom_beep(WOM_BEEP_INT1_COUNT, WOM_BEEP_MIN_GAP_INT1_MS);
        }
        else
        {
            // LOG_DEBUGF("WoM INT1: spurious edge? GPIO=%d idle=%d INT1_SRC=0x%02X", gpio_get_level(io_num), s_int1_idle_level, int_src);
        }
    }
    gpio_intr_enable(io_num);
}
// ---------------------------------------------------------------------------
// ISR handler
//
// IMPORTANT: drv_lis2dh12_read_int*_source() performs SPI I/O, which must
// NOT be called from an ISR. Interrupt source clearing is moved to the task.
// The ISR only enqueues the pin number and disables the GPIO until the task
// has processed the event, preventing burst re-triggering.
// ---------------------------------------------------------------------------

// 在 wom_lis2dh12.c 的 gpio_isr_handler 中添加延迟
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPri = pdFALSE;

    // Disable immediately to avoid repeated triggers (level mode) or burst edges.
    gpio_intr_disable(gpio_num);

    uint32_t evt = 0;
    if (gpio_num == LIS2DH12_PIN_NUM_INT1)
    {
        evt = WOM_EVT_INT1;
    }
    else if (gpio_num == LIS2DH12_PIN_NUM_INT2)
    {
        evt = WOM_EVT_INT2;
    }

    if (evt)
    {
        __atomic_fetch_or(&s_pending_events, evt, __ATOMIC_RELAXED);
        if (s_wom_task)
        {
            vTaskNotifyGiveFromISR(s_wom_task, &xHigherPri);
            if (xHigherPri)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Listener task
// ---------------------------------------------------------------------------
static void wom_listener_task(void *arg)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t pending = __atomic_exchange_n(&s_pending_events, 0, __ATOMIC_ACQ_REL);
        if (pending == 0)
        {
            continue;
        }

        // If INT1 arrives slightly earlier than INT2 (lower threshold), coalesce for a short
        // window so that an upcoming INT2 can suppress INT1 logging/handling.
        if ((pending & WOM_EVT_INT1) && !(pending & WOM_EVT_INT2))
        {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WOM_INT1_COALESCE_MS));
            pending |= __atomic_exchange_n(&s_pending_events, 0, __ATOMIC_ACQ_REL);
        }

        // INT2 has higher priority. If INT2 fires, INT1 is often co-triggered; per requirement,
        // suppress INT1 handling when an INT2 interrupt is received.
        bool int2_fired = false;
        if (pending & WOM_EVT_INT2)
        {
            const uint32_t io_num = LIS2DH12_PIN_NUM_INT2;
            uint8_t int_src = 0;
            esp_err_t ret = drv_lis2dh12_read_int2_source(&int_src);
            if (ret != ESP_OK)
            {
                LOG_ERRORF("Failed to read INT2_SRC: %s", esp_err_to_name(ret));
            }
            else if (int_src & 0x40) // IA bit set
            {
                int2_fired = true;
                const uint64_t now = esp_timer_get_time();
                s_int1_suppress_until_us = now + WOM_SUPPRESS_INT1_AFTER_INT2_US;

                LOG_WARNF("WoM INT2: posture deviation (thr=%d mg) INT2_SRC=0x%02X",
                          s_default_wom_cfg.threshold_mg_int2, int_src);
                // wom_beep(WOM_BEEP_INT2_COUNT, WOM_BEEP_MIN_GAP_INT2_MS);
            }
            else
            {
                // LOG_DEBUGF("WoM INT2: spurious edge? GPIO=%d idle=%d INT2_SRC=0x%02X", gpio_get_level(io_num), s_int2_idle_level, int_src);
            }
            gpio_intr_enable(io_num);
        }

        if (pending & WOM_EVT_INT1)
        {
            const uint64_t now = esp_timer_get_time();
            const bool suppress_int1 = int2_fired || (s_int1_suppress_until_us != 0 && now < s_int1_suppress_until_us);
            wom_int1_clear_and_rearm(suppress_int1);
        }

        // TODO: trigger health-check or wakeup sequence here
        // Any WoM event means the machine's state might have changed. Trigger the state check handler.
        if (pending) {
            xSemaphoreGive(g_state_check_semaphore);
        }
    }
}

// ---------------------------------------------------------------------------
// start_wom_lis2dh12_listener
// ---------------------------------------------------------------------------
esp_err_t start_wom_lis2dh12_listener(void)
{
    if (s_listener_started)
    {
        return ESP_OK;
    }

    BaseType_t task_created =
        xTaskCreateWithCaps(wom_listener_task, "wom_listener", 4096, NULL, 10, &s_wom_task, TASK_MEM_CAPS);
    if (task_created != pdPASS)
    {
        LOG_ERRORF("Failed to create WoM listener task");
        s_wom_task = NULL;
        return ESP_FAIL;
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIS2DH12_PIN_NUM_INT1) | (1ULL << LIS2DH12_PIN_NUM_INT2),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
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
    gpio_intr_disable(LIS2DH12_PIN_NUM_INT1);

    ret = gpio_isr_handler_add(LIS2DH12_PIN_NUM_INT2, gpio_isr_handler, (void *)LIS2DH12_PIN_NUM_INT2);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("ISR handler add INT2 failed: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
        return ret;
    }
    gpio_intr_disable(LIS2DH12_PIN_NUM_INT2);

    s_listener_started = true;
    LOG_DEBUG("WoM listener infrastructure ready.");
    return ESP_OK;
}

esp_err_t wom_lis2dh12_enable(void)
{
    if (s_wom_armed)
    {
        return ESP_OK;
    }

    esp_err_t ret = start_wom_lis2dh12_listener();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = drv_lis2dh12_enable_wom(&s_default_wom_cfg);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable WoM: %s", esp_err_to_name(ret));
        return ret;
    }

    {
        uint8_t src;
        (void)drv_lis2dh12_read_int1_source(&src);
        (void)drv_lis2dh12_read_int2_source(&src);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    s_int1_idle_level = gpio_get_level(LIS2DH12_PIN_NUM_INT1);
    s_int2_idle_level = gpio_get_level(LIS2DH12_PIN_NUM_INT2);
    s_int1_intr_type = (s_int1_idle_level == 0) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    s_int2_intr_type = (s_int2_idle_level == 0) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    LOG_DEBUGF("WoM INT idle levels: INT1=%d INT2=%d (intr INT1=%s INT2=%s)",
               s_int1_idle_level, s_int2_idle_level,
               (s_int1_intr_type == GPIO_INTR_POSEDGE) ? "posedge" : "negedge",
               (s_int2_intr_type == GPIO_INTR_POSEDGE) ? "posedge" : "negedge");

    ret = gpio_set_intr_type(LIS2DH12_PIN_NUM_INT1, s_int1_intr_type);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("gpio_set_intr_type INT1 failed: %s", esp_err_to_name(ret));
        (void)drv_lis2dh12_disable_wom();
        return ret;
    }
    ret = gpio_set_intr_type(LIS2DH12_PIN_NUM_INT2, s_int2_intr_type);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("gpio_set_intr_type INT2 failed: %s", esp_err_to_name(ret));
        (void)drv_lis2dh12_disable_wom();
        return ret;
    }

    gpio_intr_enable(LIS2DH12_PIN_NUM_INT1);
    gpio_intr_enable(LIS2DH12_PIN_NUM_INT2);
    s_wom_armed = true;

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
        const uint16_t ths_step_mg = lis2dh12_int_ths_step_mg(drv_lis2dh12_get_current_fs());
        LOG_DEBUG("WoM register dump:");
        LOG_DEBUGF("  CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL4=0x%02X CTRL5=0x%02X CTRL6=0x%02X",
                   r1, r2, r3, r4, r5, r6);
        LOG_DEBUGF("  INT1: CFG=0x%02X THS=0x%02X(%umg) DUR=0x%02X(%ums)",
                   cfg1, thr1, thr1 * ths_step_mg, dur1, dur1 * 20u);
        LOG_DEBUGF("  INT2: CFG=0x%02X THS=0x%02X(%umg) DUR=0x%02X(%ums)",
                   cfg2, thr2, thr2 * ths_step_mg, dur2, dur2 * 20u);
    }

    LOG_INFO("WoM armed");
    return ESP_OK;
}

esp_err_t wom_lis2dh12_disable(void)
{
    if (!s_listener_started)
    {
        return ESP_OK;
    }

    if (!s_wom_armed)
    {
        return ESP_OK;
    }

    gpio_intr_disable(LIS2DH12_PIN_NUM_INT1);
    gpio_intr_disable(LIS2DH12_PIN_NUM_INT2);
    (void)gpio_set_intr_type(LIS2DH12_PIN_NUM_INT1, GPIO_INTR_DISABLE);
    (void)gpio_set_intr_type(LIS2DH12_PIN_NUM_INT2, GPIO_INTR_DISABLE);
    __atomic_store_n(&s_pending_events, 0, __ATOMIC_RELEASE);
    s_int1_suppress_until_us = 0;
    s_wom_armed = false;

    esp_err_t ret = drv_lis2dh12_disable_wom();
    if (ret != ESP_OK)
    {
        LOG_WARNF("Failed to disable WoM: %s", esp_err_to_name(ret));
        return ret;
    }

    LOG_INFO("WoM disarmed");
    return ESP_OK;
}

esp_err_t wom_lis2dh12_enter_light_sleep_until_wakeup(void)
{
    esp_err_t ret = wom_lis2dh12_enable();
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT1, wom_gpio_wakeup_type_for_idle(s_int1_idle_level));
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable GPIO wakeup for INT1: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_wakeup_enable(LIS2DH12_PIN_NUM_INT2, wom_gpio_wakeup_type_for_idle(s_int2_idle_level));
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable GPIO wakeup for INT2: %s", esp_err_to_name(ret));
        (void)gpio_wakeup_disable(LIS2DH12_PIN_NUM_INT1);
        return ret;
    }
    ret = esp_sleep_enable_gpio_wakeup();
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable GPIO wakeup source: %s", esp_err_to_name(ret));
        (void)gpio_wakeup_disable(LIS2DH12_PIN_NUM_INT1);
        (void)gpio_wakeup_disable(LIS2DH12_PIN_NUM_INT2);
        return ret;
    }

    LOG_INFO("Entering light sleep, waiting for WoM wakeup");
    ret = esp_light_sleep_start();

    (void)esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    (void)gpio_wakeup_disable(LIS2DH12_PIN_NUM_INT1);
    (void)gpio_wakeup_disable(LIS2DH12_PIN_NUM_INT2);

    if (ret != ESP_OK)
    {
        LOG_ERRORF("Light sleep failed: %s", esp_err_to_name(ret));
        return ret;
    }

    LOG_INFOF("Woke from light sleep, cause=%d", esp_sleep_get_wakeup_cause());
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// wom_lis2dh12_on_wakeup
// Call this after returning from light sleep to catch edges missed during sleep.
// ---------------------------------------------------------------------------
void wom_lis2dh12_on_wakeup(void)
{
    if (!s_wom_task)
        return;
    if (s_int1_idle_level >= 0 && gpio_get_level(LIS2DH12_PIN_NUM_INT1) != s_int1_idle_level)
    {
        __atomic_fetch_or(&s_pending_events, WOM_EVT_INT1, __ATOMIC_RELAXED);
        xTaskNotifyGive(s_wom_task);
    }
    if (s_int2_idle_level >= 0 && gpio_get_level(LIS2DH12_PIN_NUM_INT2) != s_int2_idle_level)
    {
        __atomic_fetch_or(&s_pending_events, WOM_EVT_INT2, __ATOMIC_RELAXED);
        xTaskNotifyGive(s_wom_task);
    }
}
