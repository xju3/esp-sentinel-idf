#ifndef VIB_ALGO_FFT_CORE_H
#define VIB_ALGO_FFT_CORE_H

#include <stdint.h>

namespace Algo {

typedef struct
{
    float peak_freq;
    float peak_magnitude;
    float *v_complex;  // interleaved complex data: [real0, imag0, real1, imag1, ...]
    uint16_t samples;
    float sample_rate;
} fft_context_t;

// Initialize FFT context (power-of-two samples). Returns true on success.
bool fft_init(fft_context_t *ctx, uint16_t samples, float sample_rate);
// Compute FFT on input_data (length = samples). Updates peak info and spectrum buffers.
void fft_compute(fft_context_t *ctx, const float *input_data);
void fft_deinit(fft_context_t *ctx);

} // namespace Algo

#endif // VIB_ALGO_FFT_CORE_H
