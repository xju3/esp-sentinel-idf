#include "system_lock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "logger.h"

// Global mutex to ensure sequential execution of major tasks (diagnostics, OTA, etc.).
static SemaphoreHandle_t g_system_task_mutex;

void init_system_lock(void)
{
    g_system_task_mutex = xSemaphoreCreateMutex();
    if (g_system_task_mutex == NULL) {
        LOG_ERROR("Failed to create system task mutex");
    }
}

void lock_system_task(void)
{
    // LOG_DEBUG("Waiting to lock system task mutex...");
    if (xSemaphoreTake(g_system_task_mutex, portMAX_DELAY) != pdTRUE) {
        LOG_ERROR("Failed to take system task mutex");
    } else {
        // LOG_DEBUG("System task mutex locked");
    }
}

void unlock_system_task(void)
{
    if (xSemaphoreGive(g_system_task_mutex) != pdTRUE) {
        LOG_ERROR("Failed to give system task mutex");
    } else {
        // LOG_DEBUG("System task mutex unlocked");
    }
}
