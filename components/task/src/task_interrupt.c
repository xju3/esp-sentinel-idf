#include "task_interrupt.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "logger.h"
#include "drv_icm_42688_p.h"
#include "task_monitor.h"

// 假设 ICM-42688-P 的 INT1 连接到 ESP32 的 GPIO 4`
#define IMU_INT1_PIN GPIO_NUM_4
#define ESP_INTR_FLAG_DEFAULT 0
// 改为：
// TaskHandle_t imu_task_handle = NULL;

/* * 1. 中断服务例程 (ISR)
 * 注意：必须使用 IRAM_ATTR 宏，确保代码链接到内部 RAM，防止因 Flash 缓存禁用导致崩溃
 */
static void IRAM_ATTR imu_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 发送任务通知给数据处理任务
    vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);

    // 如果唤醒的任务优先级高于当前正在运行的任务，请求上下文切换
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
            int level = gpio_get_level(IMU_INT1_PIN);
            LOG_DEBUGF("收到中断！次数=%lu, INT1电平=%d", interrupt_count++, level);
            drv_icm42688_clear_wom_interrupt();
            disable_icm42688p_wom();
            if (g_wakeup_sem)
                xSemaphoreGive(g_wakeup_sem);
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