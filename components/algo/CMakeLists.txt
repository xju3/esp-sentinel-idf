idf_component_register(
    SRCS fft_core.cpp time_domain.cpp envelope.cpp iso_10816.cpp algo_rms.c algo_test.c
    INCLUDE_DIRS include .
    REQUIRES esp-dsp icm42688p config logger fs_utils
)
