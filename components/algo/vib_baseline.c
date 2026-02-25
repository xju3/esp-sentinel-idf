// algo/vib_baseline.c
// Provide storage for globals declared in vib_baseline.h

#include "vib_baseline.h"

// Global StreamBuffer handle (created/managed by app layer)
StreamBufferHandle_t g_imu_stream = NULL;

// Global baseline values (populated by app layer)
vib_baseline_t g_baseline = {0};
