idf_component_register(
    SRCS 
        "src/bsp_wifi_ap.c" 
        "src/bsp_wifi_sta.c" 
        "src/captive_dns.c" 
        "src/bsp_network.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_wifi esp_netif nvs_flash esp_event driver esp_driver_gpio esp_adc esp_timer esp_driver_ledc lwip algo util app_update)
