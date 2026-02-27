#ifndef TASK_FFT_H
#define TASK_FFT_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FFT_POINTS 1024

typedef struct {
    float x_buffer[FFT_POINTS];
    float y_buffer[FFT_POINTS];
    float z_buffer[FFT_POINTS];
    size_t write_index; // 记录写到哪里了
} fft_capture_ctx_t;
void run_fft_diagnosis(void);
#ifdef __cplusplus
}
#endif

#endif // TASK_FFT_H
