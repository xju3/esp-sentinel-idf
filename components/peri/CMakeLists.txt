idf_component_register(
    SRCS 
        "src/drv_lis2dh12.c"
        "src/drv_iis3dwb.c"
        "src/drv_t1820b.c"
        "src/imu_config.c" 
        "src/daq_iis3dwb.c"
    INCLUDE_DIRS include
    REQUIRES algo bsp
    PRIV_REQUIRES freertos esp_driver_spi esp_timer util driver
)
