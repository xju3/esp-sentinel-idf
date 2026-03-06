#include "task_interrupt.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "logger.h"
#include "drv_icm_42688_p.h"
#include "task_monitor.h"
#include "esp_timer.h"

#define WOM_DEBOUNCE_US 50000
// 假设 ICM-42688-P 的 INT1 连接到 ESP32 的 GPIO 4`
#define IMU_INT1_PIN GPIO_NUM_4
#define ESP_INTR_FLAG_DEFAULT 0
// 定义 imu_task_handle 变量
TaskHandle_t imu_task_handle = NULL;
static volatile int64_t s_last_isr_time_us = 0;

/*
 * 1. 中断服务例程 (ISR)
 * 注意：必须使用 IRAM_ATTR 宏，确保代码链接到内部 RAM，防止因 Flash 缓存禁用导致崩溃
 */
static void IRAM_ATTR imu_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();

    // 去抖：50ms 内的重复触发全部丢弃
    if (now - s_last_isr_time_us < WOM_DEBOUNCE_US)
    {
        return;
    }
    s_last_isr_time_us = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
void imu_data_processing_task(void *pvParameters)
{
    uint32_t interrupt_count = 0;
    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            int level = gpio_get_level(IMU_INT1_PIN);
            if (level == 0)
            {
                continue;
            }
            interrupt_count++;
            LOG_INFOF("IMU Interrupt received! Total count: %u", interrupt_count);
            drv_icm42688_clear_wom_interrupt();
        }
    }
}

/* * 3. 硬件与系统初始化
 */
void imu_interrupt_init(void)
{
    // A. 创建 FreeRTOS 任务，优先级建议设高一点（例如 5 或更高），确保数据及时读走
    xTaskCreate(imu_data_processing_task, "imu_task", 4096, NULL, 5, &imu_task_handle);

    // B. 配置 GPIO 引脚
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // 上升沿触发
    io_conf.pin_bit_mask = (1ULL << IMU_INT1_PIN);
    io_conf.mode = GPIO_MODE_INPUT;

    // 【核心修复】：必须开启下拉，防止 IMU 关断输出时引脚悬空
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    // C. 安装全局 GPIO 中断服务 (如果你的工程里其他地方还没调用过)
    // 注意：一个工程只能调用一次 gpio_install_isr_service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // D. 为特定 GPIO 引脚挂载 ISR 回调函数
    gpio_isr_handler_add(IMU_INT1_PIN, imu_isr_handler, NULL);
}