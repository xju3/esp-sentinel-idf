set(PERI_SRCS
    "src/drv_iis3dwb.c"
    "src/drv_ds18b20.c"
    "src/drv_4g.c"
    "src/imu_config.c"
)

# drv_lis2dh12.c is only compiled when LIS2DH12 hardware is present (-DLIS2=1)
if(LIS2)
    list(APPEND PERI_SRCS "src/drv_lis2dh12.c")
endif()

idf_component_register(
    SRCS ${PERI_SRCS}
    INCLUDE_DIRS include
    REQUIRES algo bsp hal
    PRIV_REQUIRES freertos esp_driver_gpio esp_driver_spi esp_timer util driver
)
