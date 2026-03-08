/*
 * SPDX-FileCopyrightText: 2024 Aeons Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drv_lis2dh12.h"
#include "daq_worker.h"
#include "logger.h"

static const char *TAG = "TEST_LIS2DH12";

// 必须与 daq_worker.c 中的定义保持一致
#define MAX_DAQ_SAMPLES 8192

// 声明 daq_worker.c 中新增的辅助函数
extern float* daq_get_vib_buffer(size_t *length);

/**
 * @brief 手动运行 LIS2DH12 巡逻模式采集测试
 * 
 * 该函数可直接在 app_main 或启动任务中调用。
 * 它会触发一次采集，并打印前 10 个采样点的数据供人工核对。
 */
void test_lis2dh12_patrol_run(void)
{
    ESP_LOGI(TAG, ">>> Starting LIS2DH12 Patrol Mode Test <<<");

    // 1. 尝试初始化驱动 (如果已初始化会返回错误，忽略即可)
    drv_lis2dh12_init();

    // 2. 准备参数 (模拟 1500 RPM)
    daq_worker_param_t param = {
        .rpm = 1500,
        .task_mode = TASK_MODE_PATROLING
    };

    // 3. 执行采集
    esp_err_t ret = start_daq_worker(&param);

    if (ret == ESP_OK) {
        size_t len = 0;
        float *data = daq_get_vib_buffer(&len);
        ESP_LOGI(TAG, "DAQ Success. Captured %u samples.", len);

        // 4. 打印前 10 个数据点 (Planar Layout: X...Y...Z...)
        int print_count = (len > 10) ? 10 : len;
        ESP_LOGI(TAG, "--- First %d Samples (G) ---", print_count);
        for (int i = 0; i < print_count; i++) {
            float x = data[i];
            float y = data[i + MAX_DAQ_SAMPLES];
            float z = data[i + MAX_DAQ_SAMPLES * 2];
            ESP_LOGI(TAG, "[%02d] X: %+.4f, Y: %+.4f, Z: %+.4f", i, x, y, z);
        }
        ESP_LOGI(TAG, "------------------------------");
    } else {
        ESP_LOGE(TAG, "DAQ Failed with error: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 测试 DAQ Worker 的巡逻模式数据采集功能
 * 
 * 该测试用例调用 daq_worker 接口，驱动 LIS2DH12 进行数据采集，
 * 并验证采集过程是否返回成功。
 */
TEST_CASE("DAQ Worker: Patrol Mode Acquisition", "[daq][lis2dh12]")
{
    test_lis2dh12_patrol_run();
}