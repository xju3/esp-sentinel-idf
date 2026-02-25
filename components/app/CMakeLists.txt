idf_component_register(SRCS app_baseline.c sys_daq.c task_monitor.c
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash esp_event lwip logger driver config algo mqtt)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
