#ifndef TASK_BINDING_H
#define TASK_BINDING_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Boot-time check for device binding status.
 *
 * If the device_id is empty, it queries the server. If successfully bound,
 * it saves the new device_id and goes to normal patrol sleep. If still unbound,
 * it enters a long sleep without WoM.
 */
void task_binding_check_and_sleep(void);

/**
 * @brief Handle binding update task from the server (action=3).
 *
 * Queries the server for the latest binding status, updates local config,
 * restores factory settings if unbound, and reports task completion.
 *
 * @param task_id The ID of the task triggered this action.
 */
void task_binding_execute(const char *task_id);

#ifdef __cplusplus
}
#endif

#endif // TASK_BINDING_H
