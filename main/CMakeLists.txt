idf_component_register(
    SRCS "sentinel.c"
    INCLUDE_DIRS "."
    REQUIRES config_manager network web_server nvs_flash logger icm42688p
)
