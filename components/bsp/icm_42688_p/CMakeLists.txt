idf_component_register(
    SRCS drv_icm_42688_p.c daq_icm_42688_p.c
    INCLUDE_DIRS include
    REQUIRES algo
    PRIV_REQUIRES freertos esp_driver_spi esp_timer driver logger
)
