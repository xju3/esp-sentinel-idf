idf_component_register(
    SRCS src/drv_icm_42688_p.c src/daq_icm_42688_p.c
    INCLUDE_DIRS include
    REQUIRES algo
    PRIV_REQUIRES freertos esp_driver_spi esp_timer util driver
)
