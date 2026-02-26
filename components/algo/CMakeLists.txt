# ESP-IDF Component for Predictive Maintenance Algorithms
# Pure DSP library with zero external dependencies (except esp-dsp)

idf_component_register(
    SRCS
        "src/algo_pdm.c"
        "src/algo_ingest.c"
        "src/algo_dsp_utils.c"
        "src/algo_welford.c"
        "src/algo_rms.c"
        "src/algo_kurtosis.c"
        "src/algo_envelope.c"
        "src/algo_fft.c"

    INCLUDE_DIRS
        "include"

    PRIV_INCLUDE_DIRS
        "private_include"

    REQUIRES
        esp-dsp
)
