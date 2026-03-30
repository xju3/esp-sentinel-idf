idf_component_register(
    SRCS "main.c" "init.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash algo bsp net peri task util msg
)

if(DEFINED IMU)
    # IMU is used in preprocessor numeric comparisons (e.g., #if IMU == 1),
    # so ensure it is defined as a numeric literal, not a string.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE IMU=${IMU})
endif()