idf_component_register(SRCS "wifi_ap.c" "wifi_sta.c" "ppp_4g.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif nvs_flash esp_event lwip)