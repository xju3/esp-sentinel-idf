#include "drv_icm_42688_p.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logger.h"

/* ========================================================================= *
 * 宏定义与寄存器映射 (Bank 0)
 * ========================================================================= */
#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_FIFO_CONFIG     0x16
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_FIFO_CONFIG1    0x5F
#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_FIFO_DATA       0x30

// SPI 引脚定义 (需根据实际硬件原理图修改)
#define PIN_NUM_MISO GPIO_NUM_9
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK  GPIO_NUM_12
#define PIN_NUM_CS   GPIO_NUM_10

// WoM
#define ICM42688P_REG_PWR_MGMT0     0x4E  // 电源管理寄存器 0
#define ICM42688P_REG_INT_CONFIG    0x14  // 中断引脚配置
#define ICM42688P_REG_INT_SOURCE1   0x65  // INT1 中断源路由
#define ICM42688P_REG_WOM_CONFIG    0x56  // WoM 配置 (阈值与逻辑)
#define ICM42688P_REG_SMD_CONFIG    0x57  // SMD & WoM 使能开关

// DMA 每次搬运的字节数 (必须是 6 的倍数，因为每组数据 6 字节)
// 1024 个点 * 6 字节 = 6144 字节。根据内部 SRAM 剩余情况可调整。
#define DMA_CHUNK_SAMPLES 128
#define DMA_CHUNK_BYTES   (DMA_CHUNK_SAMPLES * sizeof(imu_raw_data_t))

/* ========================================================================= *
 * 内部静态状态变量
 * ========================================================================= */
static spi_device_handle_t s_spi_handle = NULL;
static icm_data_cb_t       s_data_cb    = NULL;

// 乒乓缓存池 (必须强制分配在内部具备 DMA 能力的 SRAM 中)
static uint8_t *s_dma_ping = NULL;
static uint8_t *s_dma_pong = NULL;


static bool icm_42688_p_initialized = false;

// 根据设置转速度, 计算巡逻阶段的ODR
icm_odr_t calculate_patrol_odr(float rpm)
{
    float f_max = (rpm / 60.0f) * 10.0f; // 至少覆盖 10 倍频
    if (f_max > 2000.0f)
        return ICM_ODR_8KHZ;
    if (f_max > 500.0f)
        return ICM_ODR_4KHZ;
    if (f_max > 100.0f)
        return ICM_ODR_1KHZ;
    return ICM_ODR_200HZ; // 低速设备用低频采样，省 RAM
}

/* ========================================================================= *
 * 底层 SPI 读写辅助函数
 * ========================================================================= */
static esp_err_t icm_write_reg(uint8_t reg, uint8_t data) {
    spi_transaction_t t = {
        .length = 8,
        .cmd = reg,            // 写操作，最高位为 0
        .tx_buffer = &data     // 指向要写入的数据
    };
    return spi_device_polling_transmit(s_spi_handle, &t);
}

static esp_err_t icm_read_reg(uint8_t reg, uint8_t *data) {
    spi_transaction_t t = {
        .length = 8,
        .cmd = reg | 0x80,     // 读操作，最高位赋 1
        .rx_buffer = data
    };
    return spi_device_polling_transmit(s_spi_handle, &t);
}


/**
 * @brief 配置并启动 ICM-42688-P 的 Wake-on-Motion (WoM) 功能
 * @param threshold_mg 唤醒阈值 (单位: 毫g, 范围建议 50 ~ 255)
 */
esp_err_t enable_icm42688p_wom(uint8_t threshold_mg)
{
    esp_err_t err = ESP_OK;

    // 1. 设置中断引脚物理特性 (INT_CONFIG)
    // 假设您的硬件: INT1 引脚, 推挽输出(Push-Pull), 极性高电平有效(Active High), 脉冲模式(Pulse) 或 锁存模式(Latch)
    // Bit 1: INT1_MODE (1=Latch 锁存模式，必须读寄存器才清零; 0=Pulse 脉冲) -> 我们用 1
    // Bit 0: INT1_POLARITY (1=Active High 高电平唤醒)
    err |= icm_write_reg(ICM42688P_REG_INT_CONFIG, 0x03); 

    // 2. 切换电源模式：关闭陀螺仪，设置加速度计为低功耗模式 (LP Mode)
    // PWR_MGMT0 (0x4E): Bit [3:2] GYRO_MODE = 00 (OFF), Bit [1:0] ACCEL_MODE = 10 (LP Mode)
    err |= icm_write_reg(ICM42688P_REG_PWR_MGMT0, 0x02);
    
    // 切换电源模式后，必须等待 MEMS 稳定 (约 1-2ms)
    vTaskDelay(pdMS_TO_TICKS(5));

    // 3. 路由中断源：将 WoM 事件绑定到物理引脚 INT1
    // INT_SOURCE1 (0x65): Bit 2 (WOM_INT1_EN) = 1
    err |= icm_write_reg(ICM42688P_REG_INT_SOURCE1, (1 << 2));

    // 4. 配置 WoM 阈值与判定逻辑 (WOM_CONFIG)
    // 阈值计算：寄存器内部以 4mg 为 1 个 LSB (1 LSB = 4mg)
    // 例如传入 200mg，写入寄存器的值就是 200 / 4 = 50 (0x32)
    uint8_t threshold_val = threshold_mg / 4;
    // 假设逻辑：任一轴(X/Y/Z)超过阈值即触发 (WOM_INT_MODE = 0)
    // Bit [7:3] 填入阈值
    uint8_t wom_config_val = (threshold_val << 3); 
    err |= icm_write_reg(ICM42688P_REG_WOM_CONFIG, wom_config_val);

    // 5. 正式使能 WoM 功能 (SMD_CONFIG 或专属 APEX 使能寄存器)
    // 取决于 TDK 的寄存器版本，通常使能位在 SMD_CONFIG 的 Bit 0 (WOM_EN = 1)
    err |= icm_write_reg(ICM42688P_REG_SMD_CONFIG, 0x01); 

    if (err != ESP_OK) {
        LOG_ERROR("ICM-42688-P WoM configuration failed!");
    } else {
        LOG_INFOF("WoM Enabled. Threshold: %d mg. System ready for sleep.", threshold_mg);
    }

    return err;
}

/* ========================================================================= *
 * 阶段一：基础物理初始化 (总线与内存分配)
 * ========================================================================= */
esp_err_t drv_icm42688_init(void) {
    
    if (icm_42688_p_initialized) {
        LOG_DEBUG("ICM-42688-P already initialized!");
        return ESP_OK;
    }
    esp_err_t ret;

    // 1. 初始化 SPI 总线 (启用 DMA 控制器，通常为 SPI2 或 SPI3)
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DMA_CHUNK_BYTES + 8 // 允许的最大单次 DMA 传输量
    };
    // 使用 SPI2_HOST，并请求分配一个硬件 DMA 通道
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    // 2. 挂载 ICM-42688-P 设备到总线
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10MHz 时钟 (ICM 最大支持 24MHz)
        .mode = 0,                          // SPI Mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,                    // 允许排队等待的事务数 (对 DMA 乒乓很重要)
        .command_bits = 8,                  // 8 位命令 (寄存器地址)
    };
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi_handle);
    if (ret != ESP_OK) return ret;

    // ====================================================================
    // 3. 核心大坑：分配 DMA 专用内存 (Ping-Pong Buffers)
    // 绝对不能用普通的 malloc()！必须指定 MALLOC_CAP_DMA 和 MALLOC_CAP_INTERNAL
    // ====================================================================
    s_dma_ping = (uint8_t *)heap_caps_malloc(DMA_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_dma_pong = (uint8_t *)heap_caps_malloc(DMA_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    
    if (!s_dma_ping || !s_dma_pong) {
        LOG_ERROR("Failed to allocate DMA Ping-Pong buffers!");
        return ESP_ERR_NO_MEM;
    }

    // 4. 软复位并验证身份
    icm_write_reg(ICM_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2)); // 等待复位完成

    uint8_t who_am_i = 0;
    icm_read_reg(ICM_REG_WHO_AM_I, &who_am_i);
    if (who_am_i != 0x47) {
        LOG_ERRORF("ICM-42688-P not found! WHO_AM_I = 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    icm_42688_p_initialized = true;
    LOG_INFO("ICM-42688-P Initialized with DMA buffers.");
    return ESP_OK;
}

/* ========================================================================= *
 * 阶段二：动态业务配置 (参数下发与硬件裁剪)
 * ========================================================================= */
esp_err_t drv_icm42688_config(const icm_cfg_t *cfg) {
    if (!cfg || !s_spi_handle) return ESP_ERR_INVALID_ARG;

    // 1. 电源管理：彻底关闭陀螺仪 (Bit 3:2 = 00)，开启加速度计低噪模式 (Bit 1:0 = 11)
    icm_write_reg(ICM_REG_PWR_MGMT0, 0x03);
    //必须等待 MEMS 机械结构和 ADC 彻底稳定！
    vTaskDelay(pdMS_TO_TICKS(20)); // 给电容充电留出 1ms 时间

    // 2. 翻译并下发 ODR 和 FS
    uint8_t accel_cfg = (cfg->fs << 5) | (cfg->odr);
    icm_write_reg(ICM_REG_ACCEL_CONFIG0, accel_cfg);

    // 3. FIFO 路由配置 (必须同时开启 Accel 和 Temp 才能完美凑齐 8 字节的 Packet 1)
    // Bit 0 = 1 (Accel), Bit 2 = 1 (Temp) -> 0x01 | 0x04 = 0x05
    icm_write_reg(ICM_REG_FIFO_CONFIG1, 0x05);

    // 4. FIFO 模式配置：开启 Stream 模式 (0x40)
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x40);

    // 5. WoM 唤醒守卫分支处理
    if (cfg->enable_wom) {
        enable_icm42688p_wom(cfg->wom_thr_mg);
        LOG_INFOF("WoM Guard configured. Thr: %d mg", cfg->wom_thr_mg);
    }

    LOG_INFO("ICM-42688-P Configured. Accel Only.");
    return ESP_OK;
}

/* ========================================================================= *
 * 内部状态与任务句柄
 * ========================================================================= */
static volatile bool s_stream_running = false;
static TaskHandle_t  s_dma_task_handle = NULL;

/* ========================================================================= *
 * 核心引擎：DMA 守护任务 (The Worker Task)
 * ========================================================================= */

 static void icm_dma_worker_task(void *arg) {
    spi_transaction_t trans[2];
    spi_transaction_t *p_trans;
    esp_err_t ret;

    memset(trans, 0, sizeof(trans));

    // Ping 事务
    trans[0].length = DMA_CHUNK_BYTES * 8;
    trans[0].rx_buffer = s_dma_ping;
    trans[0].cmd = ICM_REG_FIFO_DATA | 0x80;

    // Pong 事务
    trans[1].length = DMA_CHUNK_BYTES * 8;
    trans[1].rx_buffer = s_dma_pong;
    trans[1].cmd = ICM_REG_FIFO_DATA | 0x80;

    uint8_t ping_pong_flag = 0;

    while (s_stream_running) {
        // ==============================================================
        // 【核心修复：节拍同步】
        // 在 1kHz ODR 下，IMU 产生 128 个点需要 128 毫秒。
        // 我们主动挂起当前任务 125 毫秒 (稍微提前一点点，防止 2KB 的 FIFO 溢出)。
        // 这 125 毫秒内，CPU 占用率为 0%，而硬件 IMU 正在后台默默灌装数据。
        // ==============================================================
        vTaskDelay(pdMS_TO_TICKS(125)); 

        // 选定本次使用的 DMA 缓存 (Ping 或 Pong 轮换)
        spi_transaction_t *t = (ping_pong_flag == 0) ? &trans[0] : &trans[1];
        ping_pong_flag = !ping_pong_flag;

        // 瞬间出击：派 DMA 去把这 125ms 攒下的数据一波抽干
        spi_device_queue_trans(s_spi_handle, t, portMAX_DELAY);
        
        // 阻塞等待这不到 1 毫秒的纯硬件搬运完成
        ret = spi_device_get_trans_result(s_spi_handle, &p_trans, portMAX_DELAY);
        
        if (ret == ESP_OK && s_stream_running && s_data_cb) {
            // 将纯净的数据抛给上层 (此时数据绝对不会是 -1 了)
            s_data_cb((const imu_raw_data_t *)p_trans->rx_buffer, DMA_CHUNK_SAMPLES);
        }
    }

    // 退出清理
    while (spi_device_get_trans_result(s_spi_handle, &p_trans, 0) == ESP_OK) {}
    s_dma_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ========================================================================= *
 * 对外接口：启动数据流
 * ========================================================================= */

 esp_err_t drv_icm42688_start_stream(icm_data_cb_t cb) {
    if (!cb || !s_spi_handle) return ESP_ERR_INVALID_ARG;
    if (s_stream_running) return ESP_ERR_INVALID_STATE;

    s_data_cb = cb;
    s_stream_running = true;

    // =========================================================
    // 【黄金启动序列：严格遵循 TDK 官方 FIFO 状态机】
    // =========================================================
    // 1. 先将 FIFO 切入 Bypass 模式 (关停进水阀)
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));

    // 2. 暴力冲洗 FIFO 内部残留的脏数据
    icm_write_reg(0x4B, 0x02); // SIGNAL_PATH_RESET: FIFO_FLUSH
    vTaskDelay(pdMS_TO_TICKS(2));

    // 3. 重新将 FIFO 切入 Stream 模式 (开启进水阀，开始接水！)
    icm_write_reg(ICM_REG_FIFO_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(2));

    // =========================================================

    // 创建 DMA 守护任务
    xTaskCreatePinnedToCore(
        icm_dma_worker_task, 
        "icm_dma_task", 
        4096, 
        NULL, 
        configMAX_PRIORITIES - 1,
        &s_dma_task_handle, 
        1
    );

    LOG_INFO("ICM-42688-P DMA Stream Started.");
    return ESP_OK;
}

/* ========================================================================= *
 * 对外接口：停止数据流
 * ========================================================================= */
esp_err_t drv_icm42688_stop_stream(void) {
    if (!s_stream_running) return ESP_OK;

    // 拉下总闸
    s_stream_running = false;

    // 等待守护任务安全退出 (最多等待 20 毫秒)
    // 这是一种优雅的退出机制，防止任务被强杀导致 SPI 总线死锁
    int timeout = 20; 
    while (s_dma_task_handle != NULL && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    s_data_cb = NULL;
    LOG_INFO("ICM-42688-P DMA Stream Stopped.");
    return ESP_OK;
}