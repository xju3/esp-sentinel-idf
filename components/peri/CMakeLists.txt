idf_component_register(
    SRCS 
        "src/drv_lis2dh12.c"
        "src/drv_iis3dwb.c"
        "src/drv_ds18b20.c"
        "src/drv_4g.c"
        "src/imu_config.c" 
        ""
    INCLUDE_DIRS include
    REQUIRES algo bsp hal
    PRIV_REQUIRES freertos esp_driver_gpio esp_driver_spi esp_timer util driver
)
