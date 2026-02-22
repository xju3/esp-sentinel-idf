#ifndef NET_MANAGER_H_
#define NET_MANAGER_H_

#include "esp_err.h"
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


/**
 * 扫描网络
 */
esp_err_t scan_wifi(void);

/**
 * @brief 初始化并连接到指定的 Wi-Fi 路由器 (STA 模式)
 * * @param ssid 路由器的 SSID
 * @param pass 路由器的密码
 * @return esp_err_t 返回 ESP_OK 表示连接成功
 */
esp_err_t wifi_init_sta(const char *ssid, const char *pass);

/**
 * @brief 初始化 4G 模组并启动 PPP 拨号
 * * 用于在没有 Wi-Fi 环境时，通过蜂窝网络上报数据。
 */
void ppp_init_4g(void);

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

#ifdef __cplusplus
}
#endif

#endif /* NET_MANAGER_H_ */
