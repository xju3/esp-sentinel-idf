/*
 * algo_pdm.c - Algorithm Library Module Initialization and Logging
 * 
 * Implements the unified initialization interface and logging system
 * with zero hardcoded dependencies.
 */

#include "algo_pdm.h"
#include "algo_dsp_utils.h"
#include <string.h>

/* ========================================================================= *
 * PRIVATE GLOBAL STATE (Static Storage, Zero Malloc)
 * ========================================================================= */

// Global configuration storage
static algo_config_t g_algo_config = {
    .log_cb = NULL
};

// Module initialization flag
static bool g_algo_initialized = false;

/* ========================================================================= *
 * PRIVATE LOGGING FUNCTIONS
 * ========================================================================= */

/**
 * @brief Internal logging function with NULL safety
 * @details Only calls the registered callback if it exists
 */
void _algo_log_internal(algo_log_level_t level, const char *tag, 
                        const char *fmt, ...)
{
    // Early return if no logging callback registered
    if (g_algo_config.log_cb == NULL) {
        return;
    }
    
    va_list args;
    va_start(args, fmt);
    g_algo_config.log_cb(level, tag, fmt, args);
    va_end(args);
}

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

int algo_pdm_init(const algo_config_t *config)
{
    // Check if already initialized
    if (g_algo_initialized) {
        ALGO_LOGW("algo", "Algorithm library already initialized");
        return 0; // Return success for idempotent calls
    }
    
    // Initialize with default configuration
    memset(&g_algo_config, 0, sizeof(g_algo_config));
    
    // Copy configuration if provided
    if (config != NULL) {
        // Copy only valid fields (future-proof for additional fields)
        g_algo_config.log_cb = config->log_cb;
        
        // Log initialization with configured logging
        if (g_algo_config.log_cb != NULL) {
            ALGO_LOGI("algo", "Algorithm library initialized with logging");
        } else {
            // Silent initialization - no logging callback
        }
    } else {
        // Default initialization - no logging
        ALGO_LOGI("algo", "Algorithm library initialized with default config");
    }
    
    // Mark as initialized
    g_algo_initialized = true;
    
    // Initialize submodules
    // Note: FFT initialization is deferred to first use
    // Other modules are stateless or context-based
    
    return 0; // Success
}

/* ========================================================================= *
 * INTERNAL HELPER FUNCTIONS
 * ========================================================================= */

/**
 * @brief Get the current algorithm configuration
 * @return Pointer to global configuration (read-only)
 */
const algo_config_t *_algo_get_config(void)
{
    return &g_algo_config;
}

/**
 * @brief Check if algorithm library is initialized
 * @return true if initialized, false otherwise
 */
bool _algo_is_initialized(void)
{
    return g_algo_initialized;
}

/**
 * @brief Validate FFT size (power of 2)
 * @param size FFT size to validate
 * @return true if valid, false otherwise
 */
bool _algo_validate_fft_size(size_t size)
{
    if (size == 0) {
        ALGO_LOGE("algo", "FFT size cannot be zero");
        return false;
    }
    
    // Check if size is power of 2
    if ((size & (size - 1)) != 0) {
        ALGO_LOGW("algo", "FFT size %zu is not power of 2, will be rounded up", size);
        return false;
    }
    
    return true;
}

/**
 * @brief Validate sampling frequency
 * @param fs Sampling frequency in Hz
 * @return true if valid, false otherwise
 */
bool _algo_validate_sampling_freq(float fs)
{
    if (fs <= 0.0f) {
        ALGO_LOGE("algo", "Invalid sampling frequency: %.2f Hz", fs);
        return false;
    }
    
    if (fs > 10000.0f) { // Reasonable upper limit for vibration analysis
        ALGO_LOGW("algo", "High sampling frequency: %.2f Hz", fs);
    }
    
    return true;
}

/**
 * @brief Validate axis selection
 * @param axis Axis to validate
 * @return true if valid, false otherwise
 */
bool _algo_validate_axis(algo_axis_t axis)
{
    if (axis < ALGO_AXIS_X || axis > ALGO_AXIS_Z) {
        ALGO_LOGE("algo", "Invalid axis: %d", (int)axis);
        return false;
    }
    
    return true;
}

/**
 * @brief Validate sensitivity factor
 * @param sensitivity Sensitivity factor
 * @return true if valid, false otherwise
 */
bool _algo_validate_sensitivity(float sensitivity)
{
    if (sensitivity <= 0.0f) {
        ALGO_LOGE("algo", "Invalid sensitivity: %.6f", sensitivity);
        return false;
    }
    
    return true;
}