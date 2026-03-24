/*
 * 三态判断计算过程说明（OFF / TRANSIENT / STABLE）：
 * 1) 触发检测后先置为 TRANSIENT，采集数据并按固定 1s 时间窗分窗；
 *    每窗用 Welford 统计三轴 std，再取矢量模得到 RMS，并在 RMS 最强轴上做 FFT 提取主频。
 * 2) 当历史窗填满且已超过 MIN_STABLE_DURATION_MS，计算相邻窗口变化率：
 *    R_E = |RMS_k - RMS_{k-1}| / max(RMS_{k-1}, eps)
 *    若 max(R_E) <= STABILITY_RMS_VAR_RATIO 且平均 RMS >= MIN_RMS_FOR_STABLE，则判定稳定（stable_detected）。
 * 3) 采集结束后，计算历史平均 RMS：若 avg_rms < MIN_RMS_FOR_OFF 判为 OFF；
 *    否则若 stable_detected 为真判为 STABLE；否则保持 TRANSIENT。
 */
#include "task_state_machine.h"
#include "freertos/task.h"
#include "machine_state.h"
#include "freertos/semphr.h"
#include "daq_icm_42688_p.h"
#include "algo_welford.h"
#include "algo_fft.h"
#include "algo_window.h"
#include "task_stash.h"
#include "logger.h"
#include "esp_attr.h"
#include <string.h>
#include <math.h>

// This semaphore is given by the LIS2DH12TR ISR
SemaphoreHandle_t g_state_check_semaphore;

// --- Configuration Constants for State Machine ---
#define STATE_MACHINE_ODR_HZ         3200.0f  // ODR for state analysis. Must be supported by ICM42688
#define STATE_MACHINE_WINDOW_SEC     1.0f     // Target analysis window length (seconds)
#define STATE_MACHINE_MAX_SAMPLES    8192U    // Max FFT points for state analysis (power of 2)
#define STABILITY_HISTORY_SIZE       5        // Number of consecutive windows to check for stability
#define ODR_SMOOTH_HISTORY_SIZE      5        // Number of windows for ODR sliding average
#define STABILITY_RMS_VAR_RATIO      0.15f    // Allowed relative variance for RMS to be considered stable (15%)
#define MIN_VALID_FREQ_HZ            10.0f    // Ignore frequencies below this value as potential motor speed
#define MIN_STABLE_DURATION_MS       1000     // Min duration in TRANSIENT before allowing switch to STABLE
#define MAX_TRANSITION_DURATION_S    15       // Max time to spend trying to find a stable state
#define MIN_RMS_FOR_STABLE           0.01f    // Minimum RMS acceleration (g) required for STABLE state
#define MIN_RMS_FOR_OFF              0.005f   // Maximum RMS acceleration (g) for OFF state
// Static buffers to avoid repeated heap allocations (reduce PSRAM fragmentation risk)
EXT_RAM_BSS_ATTR static float s_state_fft_input[STATE_MACHINE_MAX_SAMPLES * 3];
EXT_RAM_BSS_ATTR static float s_state_fft_output[STATE_MACHINE_MAX_SAMPLES / 2];

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
    uint32_t window_samples;
    int64_t window_start_time_us;
    float window_odr_hz;
    algo_odr_smoother_t odr_smoother;
    float* fft_input_buffer;
    float* fft_input_x;
    float* fft_input_y;
    float* fft_input_z;
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
static void init_state_context(StateAnalysisContext_t *ctx);
static void reset_window_stats(StateAnalysisContext_t *ctx);
static void start_window(StateAnalysisContext_t *ctx);
static float calc_avg_rms_history(const StateAnalysisContext_t *ctx);

typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
} AxisId_t;

static AxisId_t select_fft_axis(float std_x, float std_y, float std_z)
{
    if (std_x >= std_y && std_x >= std_z) {
        return AXIS_X;
    }
    if (std_y >= std_x && std_y >= std_z) {
        return AXIS_Y;
    }
    return AXIS_Z;
}

static uint32_t calc_window_samples(float odr_hz)
{
    if (odr_hz <= 0.0f) {
        odr_hz = STATE_MACHINE_ODR_HZ;
    }
    uint32_t n = algo_window_calc_samples(odr_hz, STATE_MACHINE_WINDOW_SEC, STATE_MACHINE_MAX_SAMPLES);
    return (n > 0U) ? n : 4U;
}

static void init_state_context(StateAnalysisContext_t *ctx)
{
    memset(ctx, 0, sizeof(StateAnalysisContext_t));
    ctx->fft_input_buffer = s_state_fft_input;
    ctx->fft_output_buffer = s_state_fft_output;
    ctx->fft_input_x = ctx->fft_input_buffer;
    ctx->fft_input_y = ctx->fft_input_buffer + STATE_MACHINE_MAX_SAMPLES;
    ctx->fft_input_z = ctx->fft_input_buffer + (STATE_MACHINE_MAX_SAMPLES * 2);
    algo_odr_smoother_init(&ctx->odr_smoother, ODR_SMOOTH_HISTORY_SIZE);
    reset_window_stats(ctx);
    ctx->window_samples = calc_window_samples(STATE_MACHINE_ODR_HZ);
    ctx->start_time_us = esp_timer_get_time();
}

static void reset_window_stats(StateAnalysisContext_t *ctx)
{
    vib_welford_1d_init(&ctx->welford_state_x);
    vib_welford_1d_init(&ctx->welford_state_y);
    vib_welford_1d_init(&ctx->welford_state_z);
    ctx->sample_idx = 0;
}

static void start_window(StateAnalysisContext_t *ctx)
{
    ctx->window_start_time_us = esp_timer_get_time();
    ctx->window_samples = calc_window_samples(ctx->window_odr_hz);
}

static float calc_avg_rms_history(const StateAnalysisContext_t *ctx)
{
    float avg_rms = 0.0f;
    int count = 0;
    for (int i = 0; i < STABILITY_HISTORY_SIZE; i++) {
        if (ctx->history[i].rms_accel > 0.0f) {
            avg_rms += ctx->history[i].rms_accel;
            count++;
        }
    }
    return (count > 0) ? (avg_rms / (float)count) : 0.0f;
}

/**
 * @brief Processes a full window of data to calculate RMS and FFT.
 */
static void process_analysis_window(StateAnalysisContext_t *ctx) {
    // 1. Finalize RMS calculation from Welford states (vector magnitude of stddev)
    float std_x = vib_welford_1d_std_sample(&ctx->welford_state_x);
    float std_y = vib_welford_1d_std_sample(&ctx->welford_state_y);
    float std_z = vib_welford_1d_std_sample(&ctx->welford_state_z);
    float current_rms = vib_3d_norm(std_x, std_y, std_z);

    // 2. Perform FFT on the axis with strongest RMS (stddev magnitude)
    AxisId_t fft_axis = select_fft_axis(std_x, std_y, std_z);
    const float *fft_input = (fft_axis == AXIS_X) ? ctx->fft_input_x :
                             (fft_axis == AXIS_Y) ? ctx->fft_input_y :
                             ctx->fft_input_z;
    uint32_t fft_len = ctx->window_samples;
    esp_err_t fft_ret = algo_fft_calculate(fft_input, ctx->fft_output_buffer, fft_len);
    float main_freq = 0.0f;
    if (fft_ret == ESP_OK) {
        float max_magnitude = 0.0f;
        uint32_t max_idx = 0;
        float odr_hz = (ctx->window_odr_hz > 0.0f) ? ctx->window_odr_hz : STATE_MACHINE_ODR_HZ;
        float freq_resolution = odr_hz / (float)fft_len;
        
        // Find peak frequency, ignoring DC and low-frequency noise
        uint32_t min_freq_idx = (uint32_t)(MIN_VALID_FREQ_HZ / freq_resolution);
        for (uint32_t i = min_freq_idx; i < fft_len / 2; i++) {
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
    reset_window_stats(ctx);

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
 * - E_H(k) must be above minimum threshold for STABLE (to avoid OFF misclassification)
 */
static bool check_stability(StateAnalysisContext_t *ctx) {
    if (!ctx->history_filled) return false;

    const float epsilon = 1e-6f;
    float max_energy_rate = 0.0f;
    float avg_rms = 0.0f;

    // Calculate adjacent window change rates for all consecutive pairs
    // Ring buffer: oldest window is at index (history_idx % STABILITY_HISTORY_SIZE)
    // Must iterate in temporal order: oldest -> newest
    // R_E(k) = |E_H(k) - E_H(k-1)| / max(E_H(k-1), epsilon)
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

        // Accumulate RMS for average
        avg_rms += curr_rms;
    }
    avg_rms /= (STABILITY_HISTORY_SIZE - 1);

    LOG_DEBUGF("Stability check: Energy Rate=%.3f (Limit %.3f), Avg RMS=%.4f (Min %.4f)", 
        max_energy_rate, STABILITY_RMS_VAR_RATIO, avg_rms, MIN_RMS_FOR_STABLE);

    if (max_energy_rate <= STABILITY_RMS_VAR_RATIO && avg_rms >= MIN_RMS_FOR_STABLE) {
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
        if (ctx->sample_idx == 0) {
            start_window(ctx);
        }
        
        // Convert raw data to G's
        const float x_g = (int16_t)__builtin_bswap16((uint16_t)data[i].x) * LSB_TO_G_16G;
        const float y_g = (int16_t)__builtin_bswap16((uint16_t)data[i].y) * LSB_TO_G_16G;
        const float z_g = (int16_t)__builtin_bswap16((uint16_t)data[i].z) * LSB_TO_G_16G;

        // Update Welford stats for RMS calculation
        vib_welford_1d_update(&ctx->welford_state_x, x_g);
        vib_welford_1d_update(&ctx->welford_state_y, y_g);
        vib_welford_1d_update(&ctx->welford_state_z, z_g);
        
        // Store per-axis samples for FFT axis selection
        ctx->fft_input_x[ctx->sample_idx] = x_g;
        ctx->fft_input_y[ctx->sample_idx] = y_g;
        ctx->fft_input_z[ctx->sample_idx] = z_g;

        ctx->sample_idx++;

        if (ctx->sample_idx >= ctx->window_samples) {
            int64_t end_us = esp_timer_get_time();
            int64_t dt_us = end_us - ctx->window_start_time_us;
            ctx->window_odr_hz = algo_odr_smoother_update(&ctx->odr_smoother, ctx->window_samples, dt_us);
            process_analysis_window(ctx);
            ctx->window_start_time_us = 0;
        }
        
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
            init_state_context(&context);

            // Configure and start DAQ capture
            icm_cfg_t capture_cfg = {
                .fs = ICM_FS_16G,
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
            // Calculate average RMS from history to determine if machine is OFF
            float avg_rms = calc_avg_rms_history(&context);

            if (avg_rms < MIN_RMS_FOR_OFF) {
                LOG_INFOF("Analysis complete. Machine state is OFF (avg RMS=%.4f g < %.4f g).", avg_rms, MIN_RMS_FOR_OFF);
                set_machine_state(STATE_OFF);
            } else if (context.stable_detected) {
                LOG_INFO("Analysis complete. Machine state is STABLE.");
                set_machine_state(STATE_STABLE);
            } else {
                LOG_WARN("Analysis complete. Machine state remains TRANSIENT (stability not reached).");
                // The state is already TRANSIENT, so no change is needed.
            }

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
