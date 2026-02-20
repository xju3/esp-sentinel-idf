idf_component_register(
    SRCS "httpd_setup.c" "api_handlers.c" "captive_dns.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_netif lwip spiffs
)
