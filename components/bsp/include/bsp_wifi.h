#ifndef CORE_WIFI_H
#define CORE_WIFI_H

#include "esp_err.h"
#include "esp_event.h"
#include "bsp_network.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动 Wi-Fi SoftAP (热点) 模式
 * * 用于设备初次配网或本地无网环境下的 Web 界面访问。
 * 默认 IP 通常为 192.168.4.1
 */
void wifi_init_softap(void);
esp_err_t wifi_stop_softap(void);


/**
 * 扫描网络
 */
esp_err_t scan_wifi(void);

/**
 * @brief 初始化并连接到指定的 Wi-Fi 路由器 (STA 模式)
 * * @param ssid 路由器的 SSID
 * @param pass 路由器的密码
 * @param connected_cb Wi-Fi连接成功后的回调函数（可选，可为NULL）
 * @return esp_err_t 返回 ESP_OK 表示连接成功
 */
esp_err_t wifi_init_sta(const char *ssid, const char *pass, cb_communication_channel_established cb);
esp_err_t wifi_stop_sta(void);


/**
 * @brief Wi-Fi 扫描完成标志
 * * 当 Wi-Fi 扫描完成时设置为 true
 */
extern volatile bool s_scan_done;



/**
 * @brief Wi-Fi 扫描开始标志
 * * 当 Wi-Fi 扫描开始时设置为 true
 */
extern volatile bool s_scan_started;

/**
 * @brief Wi-Fi 事件处理函数
 * * 处理 Wi-Fi 事件，包括扫描完成事件
 */
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);

/**
 * @brief 公共的 WiFi 初始化函数
 * * @param create_ap 是否创建 AP 网络接口
 * * @param create_sta 是否创建 STA 网络接口
 * * @return esp_err_t 返回 ESP_OK 表示初始化成功
 */
esp_err_t wifi_common_init(bool create_ap, bool create_sta);
bool wifi_common_is_started(void);
void wifi_common_mark_started(bool started);

#ifdef __cplusplus
}
#endif

#endif /* CORE_WIFI_H */
