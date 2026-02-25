idf_component_register(
    SRCS drv_icm_42688_p.c
    INCLUDE_DIRS include
    REQUIRES driver logger app fs_utils config esp_timer esp-dsp
    PRIV_REQUIRES freertos
)
