idf_component_register(
    SRCS "sentinel.c"
    INCLUDE_DIRS "."
    REQUIRES algo config wifi web nvs_flash logger icm42688p task dispatcher
)
