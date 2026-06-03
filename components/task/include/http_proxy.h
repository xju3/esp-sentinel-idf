#ifndef HTTP_PROXY_H
#define HTTP_PROXY_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 自动根据当前网络模式 (WiFi / 4G) 发起 HTTP GET 请求
 *
 * @param url 目标 URL
 * @param out_response 响应内容的指针，由内部分配内存，外部使用完毕后需 free()
 * @return esp_err_t 
 */
esp_err_t http_proxy_get(const char *url, char **out_response);

#ifdef __cplusplus
}
#endif

#endif // HTTP_PROXY_H