idf_component_register(
    SRCS "main.c" "init.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash algo bsp net peri task util msg
)
