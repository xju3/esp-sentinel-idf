idf_component_register(
    SRCS 
        "src/bsp_power.c"
        "src/bsp_wifi_ap.c" 
        "src/bsp_wifi_sta.c" 
        "src/captive_dns.c" 
        "src/bsp_4g.c"
        "src/bsp_network.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_wifi esp_netif nvs_flash esp_event driver esp_adc esp_driver_ledc lwip algo util)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
