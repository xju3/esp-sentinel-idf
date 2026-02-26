idf_component_register(SRCS "mqtt.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash esp_event lwip logger driver config mqtt)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
