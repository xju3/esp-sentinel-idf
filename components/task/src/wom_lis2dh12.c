#include "wom_lis2dh12.h"
#include "drv_lis2dh12.h"
#include "drv_tmb12a05.h"
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
// disabled, there is no hardware on production device.
static bool s_buzzer_ready = false;
static uint64_t s_last_beep_us = 0;
static uint64_t s_int1_suppress_until_us = 0;

// Bitmask for pending GPIO events (set in ISR, consumed in task)
#define WOM_EVT_INT1 (1u << 0)
#define WOM_EVT_INT2 (1u << 1)

// Buzzer patterns for WoM events
#define WOM_BEEP_INT1_COUNT 1u
#define WOM_BEEP_INT2_COUNT 3u
// TMB12A05 is an active buzzer (rated 5V, works at 3~7V). If it's powered from
// 3.3V or directly driven by a GPIO, SPL can be noticeably lower; use a longer
// beep to make events perceptible.
#define WOM_BEEP_DURATION_MS 250u
#define WOM_BEEP_INTERVAL_MS 120u
#define WOM_BEEP_MIN_GAP_INT1_MS 250u
#define WOM_BEEP_MIN_GAP_INT2_MS 50u
#define WOM_INT2_BEEP_SEQ_MS (WOM_BEEP_INT2_COUNT * WOM_BEEP_DURATION_MS + (WOM_BEEP_INT2_COUNT - 1u) * WOM_BEEP_INTERVAL_MS)
#define WOM_SUPPRESS_INT1_AFTER_INT2_MS (WOM_INT2_BEEP_SEQ_MS + 100u)
#define WOM_INT1_COALESCE_MS 30u

static void wom_beep(uint8_t count, uint32_t min_gap_ms)
{
    if (!s_buzzer_ready)
        return;

    // If the buzzer is already in use (by WoM or other modules), ignore this event.
    // This prevents overlapping beep requests from fighting each other.
    if (drv_tmb12a05_get_current_mode() != TMB12A05_MODE_OFF)
        return;

    const uint64_t now = esp_timer_get_time();
    const uint32_t seq_ms =
        (count == 0) ? 0u : (count * WOM_BEEP_DURATION_MS + (count - 1u) * WOM_BEEP_INTERVAL_MS);
    const uint32_t effective_gap_ms = (min_gap_ms > seq_ms) ? min_gap_ms : seq_ms;
    const uint64_t min_gap_us = (uint64_t)effective_gap_ms * 1000u;
    if (s_last_beep_us != 0 && (now - s_last_beep_us) < min_gap_us)
        return;

    s_last_beep_us = now;
    esp_err_t ret = drv_tmb12a05_beep_multiple(count, WOM_BEEP_DURATION_MS, WOM_BEEP_INTERVAL_MS);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Buzzer beep failed: %s", esp_err_to_name(ret));
    }
}

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
                s_int1_suppress_until_us = now + (uint64_t)WOM_SUPPRESS_INT1_AFTER_INT2_MS * 1000u;

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
    // Best-effort init: WoM can still work without a buzzer.
    // {
    //     esp_err_t ret = drv_tmb12a05_init();
    //     if (ret != ESP_OK)
    //     {
    //         LOG_ERRORF("Buzzer init failed (continuing without beep): %s", esp_err_to_name(ret));
    //         s_buzzer_ready = false;
    //     }
    //     else
    //     {
    //         s_buzzer_ready = true;
    //     }
    // }

    BaseType_t task_created =
        xTaskCreateWithCaps(wom_listener_task, "wom_listener", 4096, NULL, 10, &s_wom_task, TASK_MEM_CAPS);
    if (task_created != pdPASS)
    {
        LOG_ERRORF("Failed to create WoM listener task");
        // if (s_buzzer_ready)
        // {
        //     (void)drv_tmb12a05_deinit();
        //     s_buzzer_ready = false;
        // }
        s_wom_task = NULL;
        return ESP_FAIL;
    }

    // Configure GPIO inputs for INT1 and INT2
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, // Arm after sensor is configured + sources cleared
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LIS2DH12_PIN_NUM_INT1) | (1ULL << LIS2DH12_PIN_NUM_INT2),
        // Prefer pull-up: LIS2DH12 INT lines are commonly active-low open-drain on boards,
        // and pull-down will prevent the line from ever going high.
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install ISR service; prefer IRAM-safe allocation for stability during flash ops.
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        LOG_ERRORF("GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handlers, then immediately keep interrupts disabled until after the sensor
    // is configured and any boot-time latched sources are cleared.
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

    // Arm the sensor
    ret = drv_lis2dh12_enable_wom(&s_default_wom_cfg);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("Failed to enable WoM: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT1);
        gpio_isr_handler_remove(LIS2DH12_PIN_NUM_INT2);
        return ret;
    }

    // Clear any pending/boot-time interrupt state before enabling GPIO interrupts.
    // Reading INTx_SRC clears the corresponding latched interrupt source.
    {
        uint8_t src;
        (void)drv_lis2dh12_read_int1_source(&src);
        (void)drv_lis2dh12_read_int2_source(&src);
    }

    // Detect idle level after configuration + source clear, then select edge accordingly.
    // This makes the code robust to either active-high (push-pull) or active-low (open-drain) wiring.
    vTaskDelay(pdMS_TO_TICKS(2));
    s_int1_idle_level = gpio_get_level(LIS2DH12_PIN_NUM_INT1);
    s_int2_idle_level = gpio_get_level(LIS2DH12_PIN_NUM_INT2);
    s_int1_intr_type = (s_int1_idle_level == 0) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    s_int2_intr_type = (s_int2_idle_level == 0) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    LOG_DEBUGF("WoM INT idle levels: INT1=%d INT2=%d (intr INT1=%s INT2=%s)",
               s_int1_idle_level, s_int2_idle_level,
               (s_int1_intr_type == GPIO_INTR_POSEDGE) ? "posedge" : "negedge",
               (s_int2_intr_type == GPIO_INTR_POSEDGE) ? "posedge" : "negedge");

    // Enable GPIO interrupts after the sensor is fully configured.
    ret = gpio_set_intr_type(LIS2DH12_PIN_NUM_INT1, s_int1_intr_type);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("gpio_set_intr_type INT1 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_set_intr_type(LIS2DH12_PIN_NUM_INT2, s_int2_intr_type);
    if (ret != ESP_OK)
    {
        LOG_ERRORF("gpio_set_intr_type INT2 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_intr_enable(LIS2DH12_PIN_NUM_INT1);
    gpio_intr_enable(LIS2DH12_PIN_NUM_INT2);

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
        const uint16_t ths_step_mg = lis2dh12_int_ths_step_mg(drv_lis2dh12_get_current_fs());
        LOG_DEBUG("WoM register dump:");
        LOG_DEBUGF("  CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL4=0x%02X CTRL5=0x%02X CTRL6=0x%02X",
                   r1, r2, r3, r4, r5, r6);
        LOG_DEBUGF("  INT1: CFG=0x%02X THS=0x%02X(%umg) DUR=0x%02X(%ums)",
                   cfg1, thr1, thr1 * ths_step_mg, dur1, dur1 * 20u);
        LOG_DEBUGF("  INT2: CFG=0x%02X THS=0x%02X(%umg) DUR=0x%02X(%ums)",
                   cfg2, thr2, thr2 * ths_step_mg, dur2, dur2 * 20u);
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
