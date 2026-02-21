idf_component_register(
    SRCS fft_core.cpp time_domain.cpp envelope.cpp iso_10816.cpp
    INCLUDE_DIRS include
    REQUIRES esp-dsp
)
