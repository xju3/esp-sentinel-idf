idf_component_register(
    SRCS "src/httpd_setup.c" "src/api_handlers.c" "src/mqtt_proxy.c" "src/data_dispatcher.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_netif esp_wifi lwip spiffs network
)
