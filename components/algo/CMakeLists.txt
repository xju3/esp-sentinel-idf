idf_component_register(
    # Legacy sources kept for backward compatibility (still referenced by monitor/task_monitor.c)
    # You can remove them after migrating upper-layer call sites.
    SRCS
        algo_rms.c
        fft_core.cpp
        iso_10816.cpp

        # New modular vibration algorithm library (pure C, no malloc)
        vib_biquad.c
        vib_fft.c

        vib_baseline.c

        vib_welford_feature.c
        vib_accel_to_velocity.c
        vib_iso10816.c
        vib_fft_analyzer.c
        vib_kurtosis.c
        vib_envelope.c

    INCLUDE_DIRS include .
    REQUIRES esp-dsp icm42688p config logger fs_utils app
)
