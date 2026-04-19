#include "startup_gate.h"

#include "freertos/event_groups.h"

#define STARTUP_GATE_BIT_AP_CONNECTED BIT0
#define STARTUP_GATE_BIT_CONFIG_SAVED BIT1

static StaticEventGroup_t s_startup_gate_storage;
static EventGroupHandle_t s_startup_gate = NULL;
static volatile bool s_waiting_for_config = false;

static EventGroupHandle_t startup_gate_get_handle(void)
{
    if (s_startup_gate == NULL)
    {
        s_startup_gate = xEventGroupCreateStatic(&s_startup_gate_storage);
    }
    return s_startup_gate;
}

void startup_gate_reset(void)
{
    EventGroupHandle_t gate = startup_gate_get_handle();
    xEventGroupClearBits(gate, STARTUP_GATE_BIT_AP_CONNECTED | STARTUP_GATE_BIT_CONFIG_SAVED);
    s_waiting_for_config = false;
}

void startup_gate_set_waiting_for_config(bool waiting)
{
    startup_gate_get_handle();
    s_waiting_for_config = waiting;
}

bool startup_gate_is_waiting_for_config(void)
{
    startup_gate_get_handle();
    return s_waiting_for_config;
}

void startup_gate_mark_ap_client_connected(void)
{
    EventGroupHandle_t gate = startup_gate_get_handle();
    xEventGroupSetBits(gate, STARTUP_GATE_BIT_AP_CONNECTED);
}

void startup_gate_mark_config_completed(void)
{
    EventGroupHandle_t gate = startup_gate_get_handle();
    xEventGroupSetBits(gate, STARTUP_GATE_BIT_CONFIG_SAVED);
}

bool startup_gate_wait_for_ap_client(TickType_t wait_ticks)
{
    EventGroupHandle_t gate = startup_gate_get_handle();
    EventBits_t bits = xEventGroupWaitBits(
        gate,
        STARTUP_GATE_BIT_AP_CONNECTED,
        pdFALSE,
        pdFALSE,
        wait_ticks);
    return (bits & STARTUP_GATE_BIT_AP_CONNECTED) != 0;
}

bool startup_gate_wait_for_config_completed(TickType_t wait_ticks)
{
    EventGroupHandle_t gate = startup_gate_get_handle();
    EventBits_t bits = xEventGroupWaitBits(
        gate,
        STARTUP_GATE_BIT_CONFIG_SAVED,
        pdFALSE,
        pdFALSE,
        wait_ticks);
    return (bits & STARTUP_GATE_BIT_CONFIG_SAVED) != 0;
}
