idf_component_register(
    SRCS "sentinel.c"
    INCLUDE_DIRS "."
    REQUIRES algo config core_wifi web nvs_flash logger icm_42688_p task dispatcher
)
