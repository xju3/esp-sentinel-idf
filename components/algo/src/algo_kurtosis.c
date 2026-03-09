#include "algo_kurtosis.h"
#include "dsps_math.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ALGO_KURT";

#define MAX_PROCESS_POINTS 8192

// Use external RAM for large scratch buffer, aligned for SIMD
EXT_RAM_BSS_ATTR static float s_kurt_scratch[MAX_PROCESS_POINTS] __attribute__((aligned(16)));

static float process_axis(const float *input, uint32_t len)
{
    if (!input || len == 0) return 0.0f;
    
    if (len > MAX_PROCESS_POINTS) {
        ESP_LOGW(TAG, "Input length %lu clipped to %d", len, MAX_PROCESS_POINTS);
        len = MAX_PROCESS_POINTS;
    }

    // 1. Copy input to scratch buffer to allow in-place modification
    float *buf = s_kurt_scratch;
    memcpy(buf, input, len * sizeof(float));

    // 2. Calculate Mean
    // Note: Simple loop is often sufficient for mean, but we could use dsps if needed.
    float mean = 0.0f;
    for (int i = 0; i < len; i++) {
        mean += buf[i];
    }
    mean /= len;

    // 3. Centralize data: buf[i] = buf[i] - mean
    // dsps_addc_f32(input, output, len, C, step_in, step_out)
    dsps_addc_f32(buf, buf, len, -mean, 1, 1);

    // Now 'buf' contains (x - mean)

    // 4. Calculate Variance (2nd Moment)
    // Variance = sum((x-mean)^2) / N
    // We use dot product of buf with itself to get sum of squares
    float sum_sq = 0.0f;
    dsps_dotprod_f32(buf, buf, len, &sum_sq);
    
    float variance = sum_sq / len;

    // 5. Calculate 4th Moment
    // We need sum((x-mean)^4).
    // First, square the centralized data: buf[i] = buf[i] * buf[i]
    // Now buf will contain (x-mean)^2
    dsps_mul_f32(buf, buf, buf, len, 1, 1, 1);

    // Now calculate sum of squares of the squared data: sum(((x-mean)^2)^2) = sum((x-mean)^4)
    float sum_quad = 0.0f;
    dsps_dotprod_f32(buf, buf, len, &sum_quad);

    float m4 = sum_quad / len;

    // 6. Calculate Kurtosis
    // Kurtosis = M4 / (Variance^2)
    if (variance < 1e-9f) {
        return 0.0f; // Avoid division by zero for flat signals
    }

    return m4 / (variance * variance);
}

vib_kurtosis_t algo_kurtosis_calculate(const float *x, const float *y, const float *z, uint32_t length)
{
    vib_kurtosis_t result = {0};

    // Process axes sequentially to reuse the single scratch buffer
    if (x) {
        result.x = process_axis(x, length);
    }
    if (y) {
        result.y = process_axis(y, length);
    }
    if (z) {
        result.z = process_axis(z, length);
    }

    return result;
}