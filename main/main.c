#include "boot_flow.h"
#include "init.h"
#include "machine_state.h"

void app_main(void)
{
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    init_nvs();
    init_machine_state();

    bool ready_to_start_local_services = false;
    ESP_ERROR_CHECK(boot_flow_prepare(&ready_to_start_local_services));
    if (!ready_to_start_local_services)
    {
        return;
    }
    ESP_ERROR_CHECK(start_local_services());
}
