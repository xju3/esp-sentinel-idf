idf_component_register(
    SRCS "sentinel.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash algo bsp net peri task util test
)
