idf_component_register(
    # Legacy sources kept for backward compatibility (still referenced by monitor/task_monitor.c)
    # You can remove them after migrating upper-layer call sites.
    SRCS
        algo_rms.c
        fft_core.cpp
        iso_10816.cpp
        time_domain.cpp
        envelope.cpp

        # New modular vibration algorithm library (pure C, no malloc)
        core/vib_biquad.c
        core/vib_fft.c

        features/vib_welford_feature.c
        pipeline/vib_accel_to_velocity.c
        features/vib_iso10816.c
        features/vib_fft_analyzer.c
        features/vib_kurtosis.c
        features/vib_envelope.c

        app/vib_vibration_analyzer.c

    INCLUDE_DIRS include .
    REQUIRES esp-dsp icm42688p config logger fs_utils
)
