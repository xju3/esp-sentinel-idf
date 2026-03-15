idf_component_register(
    SRCS 
        "src/drv_icm_42688_p.c" 
        "src/drv_lis2dh12.c"
        "src/drv_iis3dwb.c"
        "src/drv_ds18b20.c"
        "src/drv_tmb12a05.c"
        "src/imu_config.c" 
        "src/daq_icm_42688_p.c" 
    INCLUDE_DIRS include
    REQUIRES algo
    PRIV_REQUIRES freertos esp_driver_spi esp_timer util driver
)
