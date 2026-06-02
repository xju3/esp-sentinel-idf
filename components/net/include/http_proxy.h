#ifndef HTTP_PROXY_H_
#define HTTP_PROXY_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 发起 HTTP GET 请求获取纯文本或 JSON 响应 (自动抹平 4G / WiFi 差异)
 * @param url 请求的完整 URL
 * @param out_response 输出指针，成功后需手动 free(*out_response)
 */
esp_err_t http_proxy_get(const char *url, char **out_response);

#ifdef __cplusplus
}
#endif
#endif /* HTTP_PROXY_H_ */