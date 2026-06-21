#ifndef TASK_STATUS_REPORT_H_
#define TASK_STATUS_REPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute the device status report task
 * 
 * Retrieves MCU temperature, 4G RSSI, LBS location, and battery voltage.
 * Packages them into JSON and posts to /sensors/status API.
 * 
 * @param task_id The task ID string from the server.
 */
void task_status_report_execute(const char *task_id);

#ifdef __cplusplus
}
#endif

#endif /* TASK_STATUS_REPORT_H_ */
