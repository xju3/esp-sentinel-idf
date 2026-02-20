#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "nvs_flash.h"
#include "network_manager.h"
#include "config_manager.h"
#include "web_server.h"
#include "logger.h"
#include "sdkconfig.h"
#include "icm4288p_baseline.h"

void app_main(void) {
    // 初始化 NVS (Wi-Fi 驱动必须用到)
    LOG_INFO("Starting device...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 加载配置
    ESP_ERROR_CHECK(config_manager_load(&g_user_config));
    
    // 若未配置，启动 AP + Captive Portal
    if (!g_user_config.is_configured) {
        wifi_init_softap();
        ESP_ERROR_CHECK(web_server_start());
        return;
    }


    // 确保ICM42688P基线存在（按设备ID），结果写入全局 g_icm_baseline
    const char *device_id = (g_user_config.device_id[0] != '\0') ? g_user_config.device_id : "default";
    esp_err_t bret = icm4288p_ensure_baseline(device_id, &g_icm_baseline);
    if (bret != ESP_OK) {
        LOG_WARNF("Baseline ensure failed: %d", bret);
    } else {
        LOG_INFOF("Baseline ready for %s | X: val=%.5f off=%.5f, Y: val=%.5f off=%.5f, Z: val=%.5f off=%.5f",
                  device_id,
                  g_icm_baseline.x.val, g_icm_baseline.x.offset,
                  g_icm_baseline.y.val, g_icm_baseline.y.offset,
                  g_icm_baseline.z.val, g_icm_baseline.z.offset);
    }


    // 已配置：按用户 Wi-Fi 信息连接 STA
    ESP_ERROR_CHECK(wifi_init_sta(g_user_config.wifi.ssid, g_user_config.wifi.pass));

#ifdef CONFIG_DEV_MODE
    // 开发模式下仍保留 Web 调试入口
    ESP_ERROR_CHECK(web_server_start());
#endif
}
