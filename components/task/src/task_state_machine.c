#include "task_state_machine.h"
#include "freertos/task.h"
#include "machine_state.h"
#include "logger.h"

// This semaphore is given by the LIS2DH12TR ISR
SemaphoreHandle_t g_state_check_semaphore;

static void task_state_check_handler(void *pvParameters)
{
    while (1) {
        // Wait indefinitely for the ISR to signal a potential state change
        if (xSemaphoreTake(g_state_check_semaphore, portMAX_DELAY) == pdTRUE) {
            LOG_INFO("State check handler triggered by interrupt.");

            // 1. Lock the system task mutex to ensure no other major task is running.
            // This will block until any ongoing patrol/diagnostic task is complete.
            lock_system_task();

            // 2. Perform the state check (placeholder logic for Phase 1)
            LOG_INFO("Starting state determination process...");

            // Set state to TRANSIENT immediately
            set_machine_state(STATE_TRANSIENT);

            // Simulate a 5-second analysis period
            LOG_INFO("Simulating analysis for 5 seconds...");
            vTaskDelay(pdMS_TO_TICKS(5000));

            // Set state to STABLE after the "analysis"
            LOG_INFO("Analysis complete. Setting state to STABLE.");
            set_machine_state(STATE_STABLE);

            // 3. Unlock the system task mutex, allowing other tasks to run.
            unlock_system_task();
        }
    }
}

void create_state_check_handler_task(void)
{
    // Create a binary semaphore to be used for signaling from the ISR.
    g_state_check_semaphore = xSemaphoreCreateBinary();
    if (g_state_check_semaphore == NULL) {
        LOG_ERROR("Failed to create state check semaphore");
        return;
    }

    // Create the handler task with a high priority to ensure it runs soon after being signaled.
    xTaskCreate(
        task_state_check_handler,
        "state_check_hdlr", // Name for debugging
        4096,               // Stack size in words
        NULL,               // Task input parameter
        10,                 // High priority
        NULL                // Task handle
    );
}
