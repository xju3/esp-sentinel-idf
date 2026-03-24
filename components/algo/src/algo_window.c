#include "algo_window.h"
#include <math.h>

#define US_PER_S 1000000.0f

static uint32_t next_pow2_u32(uint32_t n)
{
    if (n == 0U) {
        return 1U;
    }
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

void algo_odr_smoother_init(algo_odr_smoother_t *sm, uint32_t size)
{
    if (sm == NULL) {
        return;
    }
    if (size == 0U || size > ALGO_ODR_SMOOTH_MAX) {
        size = ALGO_ODR_SMOOTH_MAX;
    }
    sm->size = size;
    sm->idx = 0;
    sm->filled = false;
    sm->last_odr = 0.0f;
    for (uint32_t i = 0; i < ALGO_ODR_SMOOTH_MAX; i++) {
        sm->history[i] = 0.0f;
    }
}

float algo_odr_smoother_update(algo_odr_smoother_t *sm, uint32_t sample_count, int64_t dt_us)
{
    if (sm == NULL || sm->size == 0U || sample_count == 0U || dt_us <= 0) {
        if (sm != NULL) {
            sm->last_odr = 0.0f;
        }
        return 0.0f;
    }

    float odr_hz = (float)sample_count / ((float)dt_us / US_PER_S);
    uint32_t odr_idx = sm->idx % sm->size;
    sm->history[odr_idx] = odr_hz;
    sm->idx++;
    if (sm->idx >= sm->size) {
        sm->filled = true;
    }

    uint32_t count = sm->filled ? sm->size : sm->idx;
    float sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        sum += sm->history[i];
    }
    sm->last_odr = (count > 0U) ? (sum / (float)count) : 0.0f;
    return sm->last_odr;
}

uint32_t algo_window_calc_samples(float odr_hz, float window_sec, uint32_t max_samples)
{
    if (odr_hz <= 0.0f || window_sec <= 0.0f || max_samples == 0U) {
        return 0U;
    }

    uint32_t desired = (uint32_t)ceilf(odr_hz * window_sec);
    if (desired < 4U) {
        desired = 4U;
    }
    uint32_t n = next_pow2_u32(desired);
    if (n > max_samples) {
        n = max_samples;
    }
    return n;
}
