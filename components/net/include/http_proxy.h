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

/**
 * @brief 发起 HTTP POST 请求上传 JSON (自动抹平 4G / WiFi 差异)
 * @param url 请求的完整 URL
 * @param payload JSON 文本
 * @param out_response 输出指针，成功后需手动 free(*out_response)
 */
esp_err_t http_proxy_post_json(const char *url, const char *payload, char **out_response);

#ifdef __cplusplus
}
#endif
#endif /* HTTP_PROXY_H_ */
