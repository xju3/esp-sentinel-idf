#ifndef ALGO_WINDOW_H
#define ALGO_WINDOW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALGO_ODR_SMOOTH_MAX 8U

typedef struct {
    float history[ALGO_ODR_SMOOTH_MAX];
    uint32_t size;
    uint32_t idx;
    bool filled;
    float last_odr;
} algo_odr_smoother_t;

/**
 * @brief Initialize ODR smoother with a fixed history size.
 *
 * @param sm    Smoother instance.
 * @param size  History length (1..ALGO_ODR_SMOOTH_MAX).
 */
void algo_odr_smoother_init(algo_odr_smoother_t *sm, uint32_t size);

/**
 * @brief Update ODR estimate with a new window duration.
 *
 * @param sm            Smoother instance.
 * @param sample_count  Number of samples in the window.
 * @param dt_us         Window duration in microseconds.
 * @return float        Smoothed ODR (Hz). Returns 0 on invalid input.
 */
float algo_odr_smoother_update(algo_odr_smoother_t *sm, uint32_t sample_count, int64_t dt_us);

/**
 * @brief Calculate FFT window samples based on target ODR and time window.
 *
 * @param odr_hz       Estimated sampling rate (Hz).
 * @param window_sec   Target window duration (seconds).
 * @param max_samples  Upper bound for FFT length (power of 2).
 * @return uint32_t    FFT length (power of 2). Returns 0 on invalid input.
 */
uint32_t algo_window_calc_samples(float odr_hz, float window_sec, uint32_t max_samples);

#ifdef __cplusplus
}
#endif

#endif // ALGO_WINDOW_H
