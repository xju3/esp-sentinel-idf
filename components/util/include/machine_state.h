#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#include "freertos/FreeRTOS.h"

// Defines the possible operational states of the machine.
typedef enum {
    STATE_UNKNOWN,   // Initial state before first determination.
    STATE_OFF,       // Machine is confirmed to be off.
    STATE_TRANSIENT, // Machine is in a transitional state (starting up, shutting down, or unstable).
    STATE_STABLE     // Machine is running in a stable condition.
} machine_state_t;

/**
 * @brief Initializes the machine state management module, including mutexes.
 * Must be called once on startup.
 */
void init_machine_state(void);

/**
 * @brief Thread-safely gets the current machine state.
 * @return The current machine_state_t.
 */
machine_state_t get_machine_state(void);

/**
 * @brief Thread-safely sets the current machine state.
 * @param new_state The new state to set.
 */
void set_machine_state(machine_state_t new_state);

/**
 * @brief Locks the global mutex for major system tasks.
 *
 * Any long-running task (like diagnostics, patrol) or a state check
 * must acquire this lock before execution to ensure sequential access
 * to critical system resources.
 */
void lock_system_task(void);

/**
 * @brief Unlocks the global mutex for major system tasks.
 *
 * Must be called by the task that previously acquired the lock.
 */
void unlock_system_task(void);

#endif // MACHINE_STATE_H
