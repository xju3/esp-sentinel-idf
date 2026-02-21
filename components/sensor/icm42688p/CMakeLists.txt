idf_component_register(
    SRCS icm4288p.c icm4288p_baseline.c
    INCLUDE_DIRS include
    REQUIRES driver logger fs_utils config esp_timer esp-dsp
    PRIV_REQUIRES freertos
)
