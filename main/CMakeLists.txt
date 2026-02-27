idf_component_register(
    SRCS "sentinel.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash algo bsp network peripherals task utility
)
