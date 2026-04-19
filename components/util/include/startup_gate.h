#ifndef STARTUP_GATE_H
#define STARTUP_GATE_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void startup_gate_reset(void);
void startup_gate_set_waiting_for_config(bool waiting);
bool startup_gate_is_waiting_for_config(void);
void startup_gate_mark_ap_client_connected(void);
void startup_gate_mark_config_completed(void);
bool startup_gate_wait_for_ap_client(TickType_t wait_ticks);
bool startup_gate_wait_for_config_completed(TickType_t wait_ticks);

#ifdef __cplusplus
}
#endif

#endif // STARTUP_GATE_H
