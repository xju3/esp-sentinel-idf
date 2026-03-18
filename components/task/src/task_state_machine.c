#include "task_state_machine.h"
#include "freertos/task.h"
#include "machine_state.h"
#include "freertos/semphr.h"
#include "daq_icm_42688_p.h"
#include "imu_config.h"
#include "algo_welford.h"
#include "algo_fft.h"
#include "esp_heap_caps.h"
#include "task_stash.h"
#include "logger.h"
#include <string.h>

// This semaphore is given by the LIS2DH12TR ISR
SemaphoreHandle_t g_state_check_semaphore;

// --- Configuration Constants for State Machine ---
#define STATE_MACHINE_ODR_HZ         3200.0f  // ODR for state analysis. Must be supported by ICM42688
#define ANALYSIS_WINDOW_SAMPLES      1024     // Samples per analysis window (power of 2 for FFT)
#define STABILITY_HISTORY_SIZE       5        // Number of consecutive windows to check for stability
#define STABILITY_RMS_VAR_RATIO      0.15f    // Allowed relative variance for RMS to be considered stable (15%)
#define STABILITY_FREQ_VAR_HZ        5.0f     // Allowed absolute variance for frequency to be stable (5 Hz)
#define MIN_VALID_FREQ_HZ            10.0f    // Ignore frequencies below this value as potential motor speed
#define MIN_STABLE_DURATION_MS       1000     // Min duration in TRANSIENT before allowing switch to STABLE
#define MAX_TRANSITION_DURATION_S    15       // Max time to spend trying to find a stable state

// --- Data Structures ---

typedef struct {
    float rms_accel;
    float main_freq;
} AnalysisResult_t;

typedef struct {
    // Ring buffer for storing recent analysis results
    AnalysisResult_t history[STABILITY_HISTORY_SIZE];
    uint32_t history_idx;
    bool history_filled;

    // Buffers and state for ongoing window analysis
    uint32_t sample_idx;
    float* fft_input_buffer;
    float* fft_output_buffer;
    vib_welford_1d_t welford_state_x;
    vib_welford_1d_t welford_state_y;
    vib_welford_1d_t welford_state_z;

    // Overall state
    bool stable_detected;
    int64_t start_time_us;
} StateAnalysisContext_t;

// --- Forward Declarations ---
static void process_analysis_window(StateAnalysisContext_t *ctx);
static void state_analysis_handler(const imu_raw_data_t *data, size_t count, void *user_ctx);
static bool check_stability(StateAnalysisContext_t *ctx);

/**
 * @brief Processes a full window of data to calculate RMS and FFT.
 */
static void process_analysis_window(StateAnalysisContext_t *ctx) {
    // 1. Finalize RMS calculation from Welford states (vector magnitude of stddev)
    float std_x = vib_welford_1d_std_sample(&ctx->welford_state_x);
    float std_y = vib_welford_1d_std_sample(&ctx->welford_state_y);
    float std_z = vib_welford_1d_std_sample(&ctx->welford_state_z);
    float current_rms = vib_3d_norm(std_x, std_y, std_z);

    // 2. Perform FFT on the Z-axis data (assuming it has the clearest signal)
    esp_err_t fft_ret = algo_fft_calculate(ctx->fft_input_buffer, ctx->fft_output_buffer, ANALYSIS_WINDOW_SAMPLES);
    float main_freq = 0.0f;
    if (fft_ret == ESP_OK) {
        float max_magnitude = 0.0f;
        uint32_t max_idx = 0;
        float freq_resolution = STATE_MACHINE_ODR_HZ / ANALYSIS_WINDOW_SAMPLES;
        
        // Find peak frequency, ignoring DC and low-frequency noise
        uint32_t min_freq_idx = (uint32_t)(MIN_VALID_FREQ_HZ / freq_resolution);
        for (uint32_t i = min_freq_idx; i < ANALYSIS_WINDOW_SAMPLES / 2; i++) {
            if (ctx->fft_output_buffer[i] > max_magnitude) {
                max_magnitude = ctx->fft_output_buffer[i];
                max_idx = i;
            }
        }
        main_freq = (float)max_idx * freq_resolution;
    } else {
        LOG_ERROR("FFT calculation failed.");
    }

    // 3. Store results in history
    uint32_t current_idx = ctx->history_idx % STABILITY_HISTORY_SIZE;
    ctx->history[current_idx].rms_accel = current_rms;
    ctx->history[current_idx].main_freq = main_freq;
    ctx->history_idx++;
    if (ctx->history_idx >= STABILITY_HISTORY_SIZE) {
        ctx->history_filled = true;
    }

    LOG_DEBUGF("Window processed: RMS=%.4f g, Freq=%.2f Hz", current_rms, main_freq);

    // 4. Reset for next window
    vib_welford_1d_init(&ctx->welford_state_x);
    vib_welford_1d_init(&ctx->welford_state_y);
    vib_welford_1d_init(&ctx->welford_state_z);
    ctx->sample_idx = 0;

    // 5. Check for stability if enough time has passed and history is full
    int64_t now_us = esp_timer_get_time();
    if (ctx->history_filled && (now_us - ctx->start_time_us > MIN_STABLE_DURATION_MS * 1000)) {
        if (check_stability(ctx)) {
            ctx->stable_detected = true;
        }
    }
}

/**
 * @brief Checks if the last N windows in history are 'stable'.
 * 
 * Per specification section 7.3, stability requires:
 * - R_E(k) = |E_H(k) - E_H(k-1)| / max(E_H(k-1), epsilon) <= T_var_E
 * - R_F(k) = |F_H(k) - F_H(k-1)| / max(F_H(k-1), epsilon) <= T_var_F
 */
static bool check_stability(StateAnalysisContext_t *ctx) {
    if (!ctx->history_filled) return false;

    const float epsilon = 1e-6f;
    float max_energy_rate = 0.0f;
    float max_freq_rate = 0.0f;

    // Calculate adjacent window change rates for all consecutive pairs
    // Ring buffer: oldest window is at index (history_idx % STABILITY_HISTORY_SIZE)
    // Must iterate in temporal order: oldest -> newest
    // R_E(k) = |E_H(k) - E_H(k-1)| / max(E_H(k-1), epsilon)
    // R_F(k) = |F_H(k) - F_H(k-1)| / max(F_H(k-1), epsilon)
    int oldest_idx = ctx->history_idx % STABILITY_HISTORY_SIZE;
    for (int k = 0; k < STABILITY_HISTORY_SIZE - 1; k++) {
        int idx_prev = (oldest_idx + k) % STABILITY_HISTORY_SIZE;
        int idx_curr = (oldest_idx + k + 1) % STABILITY_HISTORY_SIZE;
        
        // Energy change rate
        float prev_rms = ctx->history[idx_prev].rms_accel;
        float curr_rms = ctx->history[idx_curr].rms_accel;
        float energy_rate = fabsf(curr_rms - prev_rms) / fmaxf(prev_rms, epsilon);
        if (energy_rate > max_energy_rate) {
            max_energy_rate = energy_rate;
        }

        // Frequency change rate
        float prev_freq = ctx->history[idx_prev].main_freq;
        float curr_freq = ctx->history[idx_curr].main_freq;
        float freq_rate = fabsf(curr_freq - prev_freq) / fmaxf(prev_freq, epsilon);
        if (freq_rate > max_freq_rate) {
            max_freq_rate = freq_rate;
        }
    }

    // Normalize frequency rate threshold: STABILITY_FREQ_VAR_HZ (Hz) / MIN_VALID_FREQ_HZ (Hz)
    // to convert from absolute Hz difference to relative rate
    float freq_rate_threshold = STABILITY_FREQ_VAR_HZ / fmaxf(MIN_VALID_FREQ_HZ, epsilon);

    LOG_DEBUGF("Stability check: Energy Rate=%.3f (Limit %.3f), Freq Rate=%.3f (Limit %.3f)", 
        max_energy_rate, STABILITY_RMS_VAR_RATIO, max_freq_rate, freq_rate_threshold);

    if (max_energy_rate <= STABILITY_RMS_VAR_RATIO && max_freq_rate <= freq_rate_threshold) {
        LOG_INFO("STABILITY DETECTED!");
        return true;
    }

    return false;
}


/**
 * @brief DAQ handler called by daq_icm_42688_p_capture with raw data chunks.
 */
static void state_analysis_handler(const imu_raw_data_t *data, size_t count, void *user_ctx) {
    StateAnalysisContext_t *ctx = (StateAnalysisContext_t *)user_ctx;
    if (!ctx || ctx->stable_detected) return;

    for (size_t i = 0; i < count; i++) {
        if (ctx->sample_idx >= ANALYSIS_WINDOW_SAMPLES) {
            process_analysis_window(ctx);
        }
        
        // Convert raw data to G's
        const float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G_16G;
        const float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G_16G;
        const float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G_16G;

        // Update Welford stats for RMS calculation
        vib_welford_1d_update(&ctx->welford_state_x, x_g);
        vib_welford_1d_update(&ctx->welford_state_y, y_g);
        vib_welford_1d_update(&ctx->welford_state_z, z_g);
        
        // For now, use Z-axis for FFT analysis
        ctx->fft_input_buffer[ctx->sample_idx] = z_g;

        ctx->sample_idx++;
        
        // If stability is found, we can stop processing further samples
        if (ctx->stable_detected) break;
    }
}

/**
 * @brief Main task for state determination. Replaces the old placeholder.
 */
static void task_state_check_handler(void *pvParameters)
{
    // Initialize FFT module once
    esp_err_t fft_init_ret = algo_fft_init();
    if (fft_init_ret != ESP_OK) {
        LOG_ERROR("Failed to initialize FFT module, state machine will be disabled.");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xSemaphoreTake(g_state_check_semaphore, portMAX_DELAY) == pdTRUE) {
            LOG_INFO("State check handler triggered. Starting state determination...");

            lock_system_task();

            set_machine_state(STATE_TRANSIENT);

            StateAnalysisContext_t context;
            memset(&context, 0, sizeof(StateAnalysisContext_t));

            // Allocate buffers from PSRAM if possible
            context.fft_input_buffer = (float*)heap_caps_malloc(ANALYSIS_WINDOW_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            context.fft_output_buffer = (float*)heap_caps_malloc((ANALYSIS_WINDOW_SAMPLES / 2) * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

            if (!context.fft_input_buffer || !context.fft_output_buffer) {
                LOG_ERROR("Failed to allocate memory for state analysis buffers.");
                heap_caps_free(context.fft_input_buffer);
                heap_caps_free(context.fft_output_buffer);
                unlock_system_task();
                continue;
            }
            
            // Initialize context
            vib_welford_1d_init(&context.welford_state_x);
            vib_welford_1d_init(&context.welford_state_y);
            vib_welford_1d_init(&context.welford_state_z);
            context.start_time_us = esp_timer_get_time();

            // Configure and start DAQ capture
            icm_cfg_t capture_cfg = {
                .fs = ICM_FS_16G, 
                // .odr = STATE_MACHINE_ODR_HZ, // Set ODR for state analysis
                .enable_wom = false
            };

            LOG_INFOF("Starting capture for %d seconds to determine stability...", MAX_TRANSITION_DURATION_S);
            esp_err_t cap_ret = daq_icm_42688_p_capture(
                &capture_cfg,
                MAX_TRANSITION_DURATION_S * 1000,
                state_analysis_handler,
                &context,
                256, // DAQ chunk size
                100  // Skip first 100ms
            );

            if (cap_ret != ESP_OK) {
                LOG_ERRORF("State analysis capture failed: %s", esp_err_to_name(cap_ret));
            }

            // Final state decision
            if (context.stable_detected) {
                LOG_INFO("Analysis complete. Machine state is STABLE.");
                set_machine_state(STATE_STABLE);
            } else {
                LOG_WARN("Analysis complete. Machine state remains TRANSIENT (stability not reached).");
                // The state is already TRANSIENT, so no change is needed.
            }

            // Cleanup
            heap_caps_free(context.fft_input_buffer);
            heap_caps_free(context.fft_output_buffer);
            unlock_system_task();
        }
    }
}

void create_state_check_handler_task(void)
{
    g_state_check_semaphore = xSemaphoreCreateBinary();
    if (g_state_check_semaphore == NULL) {
        LOG_ERROR("Failed to create state check semaphore");
        return;
    }

    xTaskCreate(
        task_state_check_handler,
        "state_check_hdlr",
        6144,  // Increased stack size for analysis buffers and logic
        NULL,
        10,
        NULL
    );
}
