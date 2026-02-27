idf_component_register(SRCS "src/core_wifi_ap.c" "src/core_wifi_sta.c" "src/captive_dns.c" "src/ppp_4g.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash esp_event driver lwip algo util)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
