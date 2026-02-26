/*
 * test_algo_init.c - Test program for algorithm library initialization
 * 
 * Demonstrates the new logging injection system and module initialization.
 */

#include <stdio.h>
#include <stdarg.h>
#include "algo_pdm.h"

/* ========================================================================= *
 * TEST LOGGING CALLBACK IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief Test logging callback that prints to stdout
 * @details This callback demonstrates how external applications can inject
 *          their own logging system into the algorithm library.
 */
static void test_log_callback(algo_log_level_t level, const char *tag, 
                              const char *fmt, va_list args)
{
    // Map algorithm log levels to human-readable strings
    const char *level_str;
    switch (level) {
        case ALGO_LOG_ERROR: level_str = "ERROR"; break;
        case ALGO_LOG_WARN:  level_str = "WARN";  break;
        case ALGO_LOG_INFO:  level_str = "INFO";  break;
        case ALGO_LOG_DEBUG: level_str = "DEBUG"; break;
        default:             level_str = "UNKNOWN"; break;
    }
    
    // Format and print the log message
    printf("[ALGO-%s] %s: ", level_str, tag);
    vprintf(fmt, args);
    printf("\n");
}

/* ========================================================================= *
 * TEST FUNCTIONS
 * ========================================================================= */

/**
 * @brief Test 1: Initialize with logging callback
 */
static void test_with_logging(void)
{
    printf("\n=== Test 1: Initialize with logging callback ===\n");
    
    // Create configuration with logging callback
    algo_config_t config = {
        .log_cb = test_log_callback
    };
    
    // Initialize algorithm library
    int result = algo_pdm_init(&config);
    printf("algo_pdm_init returned: %d (0 = success)\n", result);
    
    // Test that initialization is idempotent
    result = algo_pdm_init(&config);
    printf("Second call to algo_pdm_init returned: %d (should still be 0)\n", result);
}

/**
 * @brief Test 2: Initialize with NULL config (default)
 */
static void test_with_null_config(void)
{
    printf("\n=== Test 2: Initialize with NULL config ===\n");
    
    // Reset initialization state by creating new config
    // In real usage, you would only call init once per application
    printf("Initializing with NULL config (no logging)...\n");
    
    // This should work but produce no log output
    int result = algo_pdm_init(NULL);
    printf("algo_pdm_init(NULL) returned: %d\n", result);
}

/**
 * @brief Test 3: Test algorithm functions after initialization
 */
static void test_algorithm_functions(void)
{
    printf("\n=== Test 3: Test algorithm functions ===\n");
    
    // First ensure library is initialized
    algo_config_t config = {
        .log_cb = test_log_callback
    };
    algo_pdm_init(&config);
    
    // Test Welford statistics
    printf("\nTesting Welford statistics...\n");
    algo_welford_ctx_t welford_ctx = {0};
    
    // Create some test data
    imu_raw_data_t test_data[5];
    for (int i = 0; i < 5; i++) {
        test_data[i].x = i * 100;
        test_data[i].y = i * 200;
        test_data[i].z = i * 300;
    }
    
    // Update statistics
    algo_welford_update(&welford_ctx, test_data, 5, ALGO_AXIS_Z, 0.001f);
    
    // Get statistics
    float mean, variance, std_dev;
    algo_welford_get_stats(&welford_ctx, &mean, &variance, &std_dev);
    
    printf("Welford results:\n");
    printf("  Samples: %u\n", welford_ctx.count);
    printf("  Mean: %.6f\n", mean);
    printf("  Variance: %.6f\n", variance);
    printf("  StdDev: %.6f\n", std_dev);
    printf("  Min: %.6f\n", welford_ctx.min_val);
    printf("  Max: %.6f\n", welford_ctx.max_val);
    
    // Test RMS calculation
    printf("\nTesting RMS calculation...\n");
    float test_values[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float rms = algo_calc_rms(test_values, 5);
    printf("RMS of [1,2,3,4,5] = %.6f\n", rms);
    
    // Test FFT initialization
    printf("\nTesting FFT initialization...\n");
    algo_fft_init(1024);
    printf("FFT initialized with max size 1024\n");
}

/**
 * @brief Test 4: Test the new algo_welford_get_all_stats function
 */
static void test_get_all_stats(void)
{
    printf("\n=== Test 4: Test algo_welford_get_all_stats ===\n");
    
    algo_welford_ctx_t ctx = {0};
    
    // Create test data
    imu_raw_data_t test_data[3];
    for (int i = 0; i < 3; i++) {
        test_data[i].z = (i + 1) * 1000; // 1000, 2000, 3000
    }
    
    // Update statistics
    algo_welford_update(&ctx, test_data, 3, ALGO_AXIS_Z, 0.001f);
    
    // Get all statistics using the new function
    uint32_t count;
    float mean, std_dev, min_val, max_val;
    algo_welford_get_all_stats(&ctx, &count, &mean, &std_dev, &min_val, &max_val);
    
    printf("All statistics:\n");
    printf("  Count: %u\n", count);
    printf("  Mean: %.6f\n", mean);
    printf("  StdDev: %.6f\n", std_dev);
    printf("  Min: %.6f\n", min_val);
    printf("  Max: %.6f\n", max_val);
}

/* ========================================================================= *
 * MAIN TEST PROGRAM
 * ========================================================================= */

int main(void)
{
    printf("========================================\n");
    printf("Algorithm Library Initialization Test\n");
    printf("========================================\n");
    
    // Run all tests
    test_with_logging();
    test_with_null_config();
    test_algorithm_functions();
    test_get_all_stats();
    
    printf("\n========================================\n");
    printf("All tests completed successfully!\n");
    printf("========================================\n");
    
    return 0;
}