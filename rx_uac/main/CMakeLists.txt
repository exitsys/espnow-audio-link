idf_component_register(SRCS "alink_rx.c"
                       INCLUDE_DIRS "."
                       REQUIRES alink_common esp_wifi esp_netif esp_event nvs_flash esp_timer esp_driver_i2s esp_driver_gpio esp_ringbuf)
