idf_component_register(
    # Legacy sources kept for backward compatibility (still referenced by monitor/task_monitor.c)
    # You can remove them after migrating upper-layer call sites.
    SRCS
        algo_rms.c
        algo_fft_core.cpp
        iso_10816.cpp

        # New modular vibration algorithm library (pure C, no malloc)
        algo_biquad.c
        algo_fft.c

        algo_baseline.c

        algo_welford_feature.c
        algo_accel_to_velocity.c
        algo_iso10816.c
        algo_fft_analyzer.c
        algo_kurtosis.c
        algo_envelope.c

    INCLUDE_DIRS include .
    REQUIRES esp-dsp icm42688p config logger fs_utils
)
