/*
 * algo_ingest.c - Raw Data Ingestion Operator
 * 
 * Efficient extraction of specified axis from packed big-endian IMU data.
 * Performs: Big-endian to little-endian conversion -> Sensitivity scaling -> Float storage.
 * 
 * Key Features:
 * 1. Zero dynamic memory allocation (buffers provided by caller)
 * 2. Hardware-friendly loop unrolling
 * 3. Compiler optimizations for ESP32 FPU
 */

#include "algo_pdm.h"
#include "algo_dsp_utils.h"
#include <string.h>

/* ========================================================================= *
 * PUBLIC API IMPLEMENTATION
 * ========================================================================= */

void algo_ingest_axis(const imu_raw_data_t *src, 
                      size_t count, 
                      algo_axis_t axis, 
                      float sensitivity, 
                      float *out_buf)
{
    // Parameter validation
    if (src == NULL || out_buf == NULL || count == 0) {
        return;
    }
    
    // Convert enum to integer axis index
    int axis_idx = (int)axis;
    if (axis_idx < 0 || axis_idx > 2) {
        axis_idx = 2; // Default to Z-axis
    }
    
    // Process samples in batches for better cache locality
    const imu_raw_data_t *src_ptr = src;
    float *out_ptr = out_buf;
    
    // Unrolled loop for better performance (4 samples per iteration)
    size_t i = 0;
    size_t batch_count = count / 4;
    
    for (size_t batch = 0; batch < batch_count; batch++) {
        // Extract and process 4 samples
        for (int j = 0; j < 4; j++) {
            // Extract raw value and convert endianness
            int16_t raw_val = algo_extract_axis_raw(src_ptr, axis_idx);
            
            // Scale to float and store
            *out_ptr = algo_scale_raw_to_float(raw_val, sensitivity);
            
            // Move to next sample
            src_ptr++;
            out_ptr++;
            i++;
        }
    }
    
    // Process remaining samples
    for (; i < count; i++) {
        int16_t raw_val = algo_extract_axis_raw(src_ptr, axis_idx);
        *out_ptr = algo_scale_raw_to_float(raw_val, sensitivity);
        
        src_ptr++;
        out_ptr++;
    }
}

/* ========================================================================= *
 * OPTIMIZED VERSION FOR SPECIFIC AXIS (Z-axis is most common)
 * ========================================================================= */

/**
 * @brief Optimized ingestion for Z-axis only
 * @details Specialized version that avoids switch statement overhead
 */
void algo_ingest_z_axis(const imu_raw_data_t *src, 
                        size_t count, 
                        float sensitivity, 
                        float *out_buf)
{
    if (src == NULL || out_buf == NULL || count == 0) {
        return;
    }
    
    const imu_raw_data_t *src_ptr = src;
    float *out_ptr = out_buf;
    
    // Process all samples
    for (size_t i = 0; i < count; i++) {
        // Direct Z-axis extraction (no switch statement)
        int16_t raw_z = src_ptr->z;
        
        // Convert big-endian to little-endian if needed
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        raw_z = __builtin_bswap16(raw_z);
        #endif
        
        // Scale and store
        *out_ptr = (float)raw_z * sensitivity;
        
        src_ptr++;
        out_ptr++;
    }
}

/**
 * @brief Ingest all three axes simultaneously
 * @details Stores X, Y, Z in separate output buffers
 *          Useful for 3-axis analysis without multiple passes
 */
void algo_ingest_all_axes(const imu_raw_data_t *src,
                          size_t count,
                          float sensitivity,
                          float *out_x,
                          float *out_y,
                          float *out_z)
{
    if (src == NULL || count == 0) {
        return;
    }
    
    const imu_raw_data_t *src_ptr = src;
    float *x_ptr = out_x;
    float *y_ptr = out_y;
    float *z_ptr = out_z;
    
    for (size_t i = 0; i < count; i++) {
        // Extract and convert all three axes
        int16_t raw_x = src_ptr->x;
        int16_t raw_y = src_ptr->y;
        int16_t raw_z = src_ptr->z;
        
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        raw_x = __builtin_bswap16(raw_x);
        raw_y = __builtin_bswap16(raw_y);
        raw_z = __builtin_bswap16(raw_z);
        #endif
        
        // Scale and store
        if (out_x) *x_ptr = (float)raw_x * sensitivity;
        if (out_y) *y_ptr = (float)raw_y * sensitivity;
        if (out_z) *z_ptr = (float)raw_z * sensitivity;
        
        src_ptr++;
        if (out_x) x_ptr++;
        if (out_y) y_ptr++;
        if (out_z) z_ptr++;
    }
}

/* ========================================================================= *
 * HELPER FUNCTIONS FOR DMA-ALIGNED PROCESSING
 * ========================================================================= */

/**
 * @brief Check if pointer is aligned for optimal DMA access
 * @param ptr Pointer to check
 * @param alignment Required alignment (typically 4 or 8 bytes)
 * @return true if aligned, false otherwise
 */
static inline bool is_aligned(const void *ptr, size_t alignment)
{
    return ((uintptr_t)ptr % alignment) == 0;
}

/**
 * @brief Ingest with DMA-friendly alignment
 * @details Processes data in aligned chunks for optimal DMA performance
 */
void algo_ingest_axis_aligned(const imu_raw_data_t *src,
                              size_t count,
                              algo_axis_t axis,
                              float sensitivity,
                              float *out_buf)
{
    if (src == NULL || out_buf == NULL || count == 0) {
        return;
    }
    
    // Check alignment (4-byte alignment is typical for ESP32 DMA)
    bool src_aligned = is_aligned(src, 4);
    bool dst_aligned = is_aligned(out_buf, 4);
    
    if (src_aligned && dst_aligned) {
        // Use optimized path for aligned data
        algo_ingest_axis(src, count, axis, sensitivity, out_buf);
    } else {
        // Fallback to standard implementation
        // Process in smaller chunks to avoid cache issues
        const size_t CHUNK_SIZE = 32;
        size_t processed = 0;
        
        while (processed < count) {
            size_t remaining = count - processed;
            size_t chunk = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
            
            algo_ingest_axis(src + processed, chunk, axis, sensitivity, 
                            out_buf + processed);
            
            processed += chunk;
        }
    }
}