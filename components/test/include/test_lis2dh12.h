/*
 * SPDX-FileCopyrightText: 2024 Aeons Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEST_LIS2DH12_H
#define TEST_LIS2DH12_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 手动运行 LIS2DH12 巡逻模式采集测试
 * @return esp_err_t 采集过程的错误码，ESP_OK 表示成功。
 */
esp_err_t test_lis2dh12_patrol_run(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_LIS2DH12_H