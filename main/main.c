#include "nvs_flash.h"
#include "network_manager.h"
// ... 引入你的组件头文件

void app_main(void) {
    // 第一步：初始化 NVS (Wi-Fi 驱动必须用到)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 第二步：启动 AP 热点
    wifi_init_softap();

    // 第三步：挂载 10MB 的 storage 分区 (SPIFFS)
    // mount_spiffs(); 

    // 第四步：启动 Web Server (此时系统已经有了 192.168.4.1 的本地 IP)
    // start_webserver(); 
}