#include "machine_state.h"
#include "freertos/semphr.h"
#include "logger.h"

// Volatile to prevent compiler optimizations on multi-threaded access.
static volatile machine_state_t g_current_machine_state = STATE_UNKNOWN;

// Mutex to protect access to g_current_machine_state.
static SemaphoreHandle_t g_machine_state_mutex;

// Global mutex to ensure sequential execution of major tasks (diagnostics, state checks, etc.).
static SemaphoreHandle_t g_system_task_mutex;


void init_machine_state(void)
{
    g_machine_state_mutex = xSemaphoreCreateMutex();
    g_system_task_mutex = xSemaphoreCreateMutex();
    if (g_machine_state_mutex == NULL || g_system_task_mutex == NULL) {
        LOG_ERROR("Failed to create machine state mutexes");
        // In a real scenario, you might want to handle this more gracefully.
        // For now, we'll let it fail and subsequent operations will likely crash.
    }
    set_machine_state(STATE_OFF); // Start with a defined OFF state
}

machine_state_t get_machine_state(void)
{
    machine_state_t state;
    if (xSemaphoreTake(g_machine_state_mutex, portMAX_DELAY) == pdTRUE) {
        state = g_current_machine_state;
        xSemaphoreGive(g_machine_state_mutex);
    } else {
        // This should not happen if the mutex is created correctly.
        LOG_ERROR("Failed to take machine state mutex");
        state = STATE_UNKNOWN;
    }
    return state;
}

void set_machine_state(machine_state_t new_state)
{
    if (xSemaphoreTake(g_machine_state_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_current_machine_state != new_state) {
            g_current_machine_state = new_state;
            LOG_INFOF("Machine state changed to %d", new_state);
        }
        xSemaphoreGive(g_machine_state_mutex);
    } else {
        LOG_ERROR("Failed to take machine state mutex for setting state");
    }
}

void lock_system_task(void)
{
    LOG_DEBUG("Waiting to lock system task mutex...");
    if (xSemaphoreTake(g_system_task_mutex, portMAX_DELAY) != pdTRUE) {
        LOG_ERROR("Failed to take system task mutex");
    } else {
        LOG_DEBUG("System task mutex locked");
    }
}

void unlock_system_task(void)
{
    if (xSemaphoreGive(g_system_task_mutex) != pdTRUE) {
        LOG_ERROR("Failed to give system task mutex");
    } else {
        LOG_DEBUG("System task mutex unlocked");
    }
}
