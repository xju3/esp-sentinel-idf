# 工业电机振动边缘诊断网关算法库架构设计

## 目录文件树（最佳解耦逻辑）

```
components/algo/
├── CMakeLists.txt                    # 组件构建配置
├── include/                          # 对外暴露的公共头文件
│   ├── algo_pdm.h                    # 顶层API头文件（唯一外部接口）
│   ├── algo_types.h                  # 公共数据类型定义
│   └── algo_err.h                    # 错误码定义
├── private_include/                  # 内部头文件（不对外暴露）
│   ├── algo_math.h                   # 硬件隔离抽象层（核心！）
│   ├── algo_dsp_utils.h              # 公共DSP算子层
│   ├── algo_ingestor.h               # 数据摄入引擎
│   ├── algo_envelope_core.h          # 包络分析核心
│   ├── algo_welford_core.h           # Welford算法核心
│   ├── algo_fft_core.h               # FFT算法核心
│   ├── algo_rms_core.h               # RMS算法核心
│   └── algo_kurtosis_core.h          # 峭度算法核心
├── src/                              # 实现文件
│   ├── algo_pdm.c                    # 顶层API实现（日志注册等）
│   ├── algo_math.c                   # 硬件隔离实现（ESP32/PC路由）
│   ├── algo_ingestor.c               # 数据摄入算子实现
│   ├── algo_envelope.c               # 包络分析实现
│   ├── algo_welford.c                # Welford算法实现
│   ├── algo_fft.c                    # FFT算法实现
│   ├── algo_rms.c                    # RMS算法实现
│   ├── algo_kurtosis.c               # 峭度算法实现
│   └── algo_dsp_utils.c              # 公共DSP算子实现
└── test/                             # 单元测试（可选）
    ├── test_algo_ingestor.c
    ├── test_algo_envelope.c
    └── test_algo_math.c
```

## 架构分层说明

### 1. 应用接口层 (Top-Level API) - `algo_pdm.h`
- **唯一外部接口**：外部模块只能包含此头文件
- **零业务耦合**：不包含任何项目内部头文件
- **纯函数接口**：所有函数均为纯数学运算
- **内存外部管理**：所有缓冲区由调用方分配

### 2. 硬件隔离层 (Algorithm HAL) - `algo_math.h/c`
- **核心解耦层**：隔离ESP32硬件加速与PC纯C实现
- **条件编译路由**：使用 `#ifdef ESP_PLATFORM` 路由到 `esp-dsp`
- **统一接口**：提供统一的数学函数接口
- **占位符设计**：PC端使用标准 `<math.h>` 或提示接入KissFFT

### 3. 公共算子层 (Shared DSP Utils) - `algo_dsp_utils.h/c`
- **内部复用工具**：各算法模块共享的基础算子
- **数据预处理**：高通滤波、去直流、信号整流等
- **统计计算**：均值、方差、标准差等基础统计
- **信号变换**：积分、微分、包络检波等

### 4. 数据摄入层 (Raw Ingestor) - `algo_ingestor.h/c`
- **高效解包引擎**：处理 `imu_raw_data_t` 数组
- **大端转换**：使用 `__builtin_bswap16` 转换字节序
- **物理单位转换**：乘以LSB_TO_G转换为float
- **零拷贝设计**：直接输出到调用方提供的缓冲区

### 5. 算法核心层 (Algorithm Cores)
- **Welford核心**：在线流式计算均值/方差
- **RMS核心**：均方根特征提取
- **FFT核心**：频域特征提取（通过硬件隔离层）
- **Kurtosis核心**：峭度计算
- **Envelope核心**：包络解调分析

## 硬性约束验证

### ✅ 绝对零业务耦合
- `algo_pdm.h` 仅包含标准C头文件
- 内部模块不包含任何项目业务头文件
- 完全独立的数学黑盒

### ✅ 零动态内存分配
- 所有函数接受外部分配的缓冲区指针
- 使用上下文结构体存储状态
- 无 `malloc`/`calloc`/`free` 调用

### ✅ 完全解耦的日志系统
- 定义 `algo_log_cb_t` 回调函数类型
- 提供 `algo_register_log_callback()` 注册接口
- 内部使用宏进行空指针安全检查

### ✅ 高效数据解包与摄入
- 专用 `algo_ingestor` 模块处理原始数据
- 内联函数进行大端到小端转换
- 批量处理优化缓存性能

### ✅ 硬件加速的隔离抽象
- `algo_math.h` 提供统一数学接口
- `algo_math.c` 实现平台路由逻辑
- ESP32平台使用 `esp-dsp` 硬件加速
- PC平台使用标准数学库或占位符

## 数据流示例：包络分析

```
原始数据流: imu_raw_data_t[] → algo_ingestor → float[]
              ↓
        高通滤波去直流 (algo_math_hpf)
              ↓
        绝对值整流 (algo_rectify)
              ↓
        低通滤波提取包络 (algo_math_lpf)
              ↓
        输出: float[] (包络信号)
```

## 跨平台支持

### ESP32-S3 (ESP-IDF)
- 使用 `esp-dsp` 硬件加速库
- 启用FPU和SIMD指令优化
- DMA友好的内存对齐

### PC仿真环境 (Mac/Windows/Linux)
- 使用标准C数学库 `<math.h>`
- 可接入KissFFT进行FFT计算
- 支持CWRU等公开数据集验证

## 性能优化

1. **内存零拷贝**：所有操作在原缓冲区或调用方提供的缓冲区进行
2. **批量处理**：使用SIMD指令进行向量化计算
3. **缓存友好**：数据局部性优化，减少缓存失效
4. **内联关键路径**：高频调用函数使用 `static inline`
5. **循环展开**：关键循环手动展开优化