# Sentinel 固件 (EC800Z-CN 分支)

Sentinel 是一个基于 ESP32 架构的工业级高精度振动与状态监测物联网终端固件。该固件主要用于设备健康监测、预见性维护以及异常震动告警场景。此分支专为搭配 **Quectel EC800Z-CN 4G 模组** 的硬件版本设计。

## 🌟 核心特性

- **双网络支持**：支持 Wi-Fi 及 4G (Quectel EC800Z-CN) 两种网络通信模式，支持 NTP 和 4G 基站自动对时。
- **高精度振动数据采集**：内置驱动支持 **IIS3DWB** 工业级超宽带加速度计，实现高频、高精度的数据采集 (DAQ)。
- **WoM 极低功耗唤醒**：利用 **LIS2DH12** (Wake-on-Motion) 实现冲击与运动检测，支持硬件中断将 MCU 从深睡中极速唤醒。
- **端侧边缘计算 (Edge DSP)**：内置多种机器健康特征提取算法，能够在设备端侧直接计算 RMS、峰值 (Peak)、波峰因数 (Crest Factor) 和脉冲指标 (Impulse Factor) 并上报。
- **温度监测**：原生支持 **DS18B20** 单总线温度传感器。
- **极低功耗管理**：
  - 支持“冷启动”与“热唤醒”双模式，热唤醒下快速保持 RTC 时间以省去冗长的网络对时步骤。
  - 根据调度器动态规划睡眠时间，任务执行完毕后进入深度睡眠 (Deep Sleep)。
  - 支持“密集诊断模式”(Dense Diagnostic Mode)，在触发警报或异常时可临时动态提高采样和上报频率。
- **配置与管理**：
  - 基于 SPIFFS 文件系统内置了本地 Web 服务器配置界面 (`resources/system/www`)。
  - 支持云端任务下发与分块 OTA (Over-The-Air) 断点续传固件升级。

## 📁 目录结构

* `components/`
  * `algo/` - 边缘计算与信号处理算法库 (RMS 特征等)
  * `bsp/` - 板级支持包 (4G EC800Z-CN, Wi-Fi 等)
  * `msg/` - 消息中间件、队列与数据结构
  * `net/` - HTTP 及云端网络通信服务
  * `peri/` - 核心外设驱动 (IIS3DWB, LIS2DH12, DS18B20 等)
  * `task/` - 业务逻辑与后台任务 (RMS, HTTP, OTA, DAQ, 网络保活)
  * `util/` - 通用工具函数与日志组件
* `main/` - 应用程序入口及主状态机 (`main.c`)
* `resources/` - 静态资源文件 (Web UI, SPIFFS 默认配置)

## 🚀 快速上手

### 环境准备

确保已安装 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/index.html) 或适配该项目的对应版本环境。

### 编译与烧录

1. 配置项目参数：
   ```bash
   idf.py menuconfig
   ```
2. 编译并烧录固件（CMake 脚本会自动将前端 UI 和配置打包为 SPIFFS 文件系统镜像并烧录）：
   ```bash
   # 编译
   idf.py build
   # 烧录并打开串口监视器
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## 📡 4G 模组说明 (EC800Z-CN)

该分支下的网络底层逻辑（主要位于 `bsp_4g.c` 中）完全基于移远通信 (Quectel) EC800Z-CN 模组的 AT 指令集开发：
- 硬件控制上使用该模组特定的时序（如 `600ms` 的开机脉冲）和特定的指令（如 `AT+QPOWD=1` 进行关机）。
- 网络栈使用了专有的 `AT+QHTTP*` 系列指令（如 `AT+QHTTPPOST` 和 `AT+QHTTPGET`）实现 JSON 上报及 OTA 固件拉取。

*注：如果您使用的是其它 4G 模组硬件（例如 SIMCom A7670C），由于底层 HTTP 协议栈实现不同，请切换至相应的适配分支编译固件。*

---
© Sentinel Team. All Rights Reserved.
