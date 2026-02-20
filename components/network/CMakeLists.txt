idf_component_register(SRCS "captive_dns.c" "wifi_ap.c" "wifi_sta.c" "ppp_4g.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash esp_event lwip logger)

if(DEFINED DEVICE_ID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID="${DEVICE_ID}")
endif()
