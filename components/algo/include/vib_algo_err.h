// vib_algo_err.h
// 统一错误码/返回状态定义（纯C，可在MCU上运行）
//
// 设计目标：
//  - 统一各算法模块返回值，便于上层 pipeline/app 组合。
//  - 不依赖 malloc/free。
//
// 使用场景：
//  - vib_analyze_window() 及各子模块返回 vib_algo_err_t。

#ifndef VIB_ALGO_ERR_H
#define VIB_ALGO_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VIB_ALGO_OK = 0,

    // 参数问题
    VIB_ALGO_BAD_ARGS = -1,

    // 不支持的FFT长度/采样率等
    VIB_ALGO_UNSUPPORTED_N = -2,

    // 运行时资源不足（本库不使用malloc，但可能遇到缓冲区不够等）
    VIB_ALGO_NO_RESOURCE = -3,

    // 当前配置不支持/未启用
    VIB_ALGO_UNSUPPORTED = -4,
} vib_algo_err_t;

#ifdef __cplusplus
}
#endif

#endif // VIB_ALGO_ERR_H
