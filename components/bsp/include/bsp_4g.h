#ifndef PPP_4G_H_
#define PPP_4G_H_

#include "esp_err.h"
#include "esp_event.h"
#include "bsp_network.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 初始化 4G 模组并启动 PPP 拨号
 * * 用于在没有 Wi-Fi 环境时，通过蜂窝网络上报数据。
 */

esp_err_t init_ppp_4g(cb_communication_channel_established cb);
esp_err_t shutdown_ppp_4g(void);

#ifdef __cplusplus
}
#endif

#endif /* PPP_4G_H_ */
