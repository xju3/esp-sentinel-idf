# ESP-IDF Component for Predictive Maintenance Algorithms
# Pure DSP library with zero external dependencies (except esp-dsp)

idf_component_register(
    SRCS
        "src/algo_sampling.c"

    INCLUDE_DIRS
        "include"

    PRIV_INCLUDE_DIRS
        "private"

    REQUIRES
        esp-dsp
)