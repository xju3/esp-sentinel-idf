idf_component_register(
    SRCS 
        "src/httpd_setup.c" 
        "src/api_handlers.c" 
        "src/http_proxy.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_netif esp_wifi lwip spiffs net util bsp peri
)
