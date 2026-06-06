idf_component_register(
    SRCS 
        "src/httpd_setup.c" 
        "src/api_handlers.c" 
        "src/mqtt_proxy.c" 
        "src/http_proxy.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_netif esp_wifi mqtt lwip spiffs net util bsp peri esp_http_client
)

if(DEFINED SN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SN=${SN})
endif()
