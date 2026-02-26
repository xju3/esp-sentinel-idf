// algo_err.h
// 统一错误码/返回状态定义（纯C，可在MCU上运行）
//
// 注意：历史代码大量使用 VIB_ALGO_* 常量名。为了让“文件名从 vib_* 迁移为 algo_*”
// 的同时不强制改动所有调用点，这里保留 VIB_ALGO_* 作为主常量名，并提供 ALGO_* 别名。

#ifndef ALGO_ERR_H
#define ALGO_ERR_H

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

// ---- Optional aliases: ALGO_* ----
// 用宏提供 ALGO_* 别名，避免枚举重复定义。
#ifndef ALGO_OK
#define ALGO_OK VIB_ALGO_OK
#endif
#ifndef ALGO_BAD_ARGS
#define ALGO_BAD_ARGS VIB_ALGO_BAD_ARGS
#endif
#ifndef ALGO_UNSUPPORTED_N
#define ALGO_UNSUPPORTED_N VIB_ALGO_UNSUPPORTED_N
#endif
#ifndef ALGO_NO_RESOURCE
#define ALGO_NO_RESOURCE VIB_ALGO_NO_RESOURCE
#endif
#ifndef ALGO_UNSUPPORTED
#define ALGO_UNSUPPORTED VIB_ALGO_UNSUPPORTED
#endif

#ifdef __cplusplus
}
#endif

#endif // ALGO_ERR_H
