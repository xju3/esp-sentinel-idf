idf_component_register(
    SRCS "httpd_setup.c" "api_handlers.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_netif esp_wifi lwip spiffs config fs_utils logger wifi
)
