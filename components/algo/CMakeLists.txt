# ESP-IDF Component for Predictive Maintenance Algorithms
# Pure DSP library with zero external dependencies (except esp-dsp)

idf_component_register(
    SRCS
        "src/algo_welford.c"
        "src/algo_rms.c"
        "src/algo_fft.c"
        "src/algo_kurtosis.c"
        "src/algo_envelope.c"
    INCLUDE_DIRS
        "include"

    REQUIRES
        esp-dsp
)
