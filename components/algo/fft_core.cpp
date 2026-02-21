#include "fft_core.h"
#include <stdlib.h>
#include <math.h>
#include "esp_dsp.h"

namespace Algo
{

bool fft_init(fft_context_t *ctx, uint16_t samples, float sample_rate)
{
    if (!ctx || samples == 0 || (samples & (samples - 1)) != 0)
        return false; // need power of two

    ctx->samples = samples;
    ctx->sample_rate = sample_rate;
    ctx->peak_freq = 0;
    ctx->peak_magnitude = 0;
    // Allocate memory for complex data (interleaved: real, imag, real, imag...)
    ctx->v_complex = (float *)calloc(samples * 2, sizeof(float));
    if (!ctx->v_complex)
        return false;

    // Initialize esp-dsp FFT tables once
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    return true;
}

void fft_deinit(fft_context_t *ctx)
{
    if (!ctx)
        return;
    if (ctx->v_complex)
        free(ctx->v_complex);
    ctx->v_complex = NULL;
}

void fft_compute(fft_context_t *ctx, const float *input_data)
{
    if (!ctx || !input_data || !ctx->v_complex)
        return;

    // Remove DC component
    float sum = 0;
    for (uint16_t i = 0; i < ctx->samples; i++)
        sum += input_data[i];
    float mean = sum / (float)ctx->samples;

    // Prepare complex data (interleaved format)
    for (uint16_t i = 0; i < ctx->samples; i++)
    {
        ctx->v_complex[i * 2] = input_data[i] - mean;     // real part
        ctx->v_complex[i * 2 + 1] = 0.0f;                 // imag part
    }

    // esp-dsp in-place complex FFT (new API expects interleaved data)
    dsps_fft2r_fc32(ctx->v_complex, ctx->samples);
    dsps_bit_rev_fc32(ctx->v_complex, ctx->samples);
    dsps_cplx2real_fc32(ctx->v_complex, ctx->samples);

    // Find peak magnitude and frequency (skip DC bin)
    // After cplx2real, the first half contains real parts
    float max_val = 0;
    uint16_t max_idx = 1;
    uint16_t half = ctx->samples / 2;
    for (uint16_t i = 1; i < half; i++)
    {
        float mag = fabsf(ctx->v_complex[i * 2]);
        if (mag > max_val)
        {
            max_val = mag;
            max_idx = i;
        }
    }
    ctx->peak_magnitude = max_val;
    ctx->peak_freq = ((float)max_idx * ctx->sample_rate) / (float)ctx->samples;
}

} // namespace Algo

