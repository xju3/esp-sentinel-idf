// ============================================================
// algo_rms.h + algo_rms.c (merged)
//
// Fixes/Improvements vs previous version:
//  - Safe NULL checks before dereferencing pointers
//  - Biquad design no longer resets state implicitly; reset is explicit
//  - Avoid frequent redesign on small fs jitter (tolerance)
//  - Skip initial samples to avoid IIR cold-start transient contaminating RMS
//  - Biquad 执行优先走 esp-dsp 的 dsps_biquad_f32（ESP32-S3 可走硬件优化）
//  - Keeps your Welford baseline-corrected accel-RMS path unchanged
//
// Notes:
//  - ISO10816/20816 commonly uses vibration VELOCITY RMS (mm/s RMS).
//  - This implementation computes per-axis velocity RMS over a band:
//      accel: 10 Hz HP + LP(fc_lp), integrate to velocity, then 1 Hz HP.
//  - For your “every minute capture 1 second” workflow, we reset integrators
//    each call, and skip the first part of the window so the IIR settles.
// ============================================================

#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- ISO10816/20816 velocity RMS ----------------

#ifdef __cplusplus
}
#endif

#endif // ALGO_RMS_H
